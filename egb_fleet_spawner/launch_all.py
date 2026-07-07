# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 The EgbFleetAdapter Authors

"""Bazel-native equivalent of fleet_spawner_with_bridge.launch.py.

Brings up the three processes the original ros2 launch starts:
  1. egb_fleet_spawner       (Python service node)
  2. fleet_service_bridge    (C++ service bridge across DDS domains)
  3. domain_bridge_node      (C++ topic bridge for /clock)

Required env: FOREIGN_DOMAIN_ID, ROS_DOMAIN_ID
"""

import os
import signal
import subprocess
import sys
import tempfile

from python.runfiles import Runfiles


def _must(env_var: str) -> str:
    val = os.environ.get(env_var)
    if not val:
        sys.exit(
            f"ERROR: {env_var} required "
            "(e.g. FOREIGN_DOMAIN_ID=45 ROS_DOMAIN_ID=41)"
        )
    return val


def main() -> int:
    foreign_domain = _must("FOREIGN_DOMAIN_ID")
    ros_domain = _must("ROS_DOMAIN_ID")

    r = Runfiles.Create()
    spawner = r.Rlocation("_main/egb_fleet_spawner/spawner")
    service_bridge = r.Rlocation("_main/egb_fleet_spawner/fleet_service_bridge")
    # bazel 8 canonical repo names use + not ~
    domain_bridge_node = r.Rlocation(
        "+non_module_deps+domain_bridge/domain_bridge_node"
    )
    # Deployment override seam, mirrors the EGB_FLEET_ADAPTER_NODE pattern below.
    template_cfg = os.environ.get("EGB_FLEET_CONFIG") or r.Rlocation(
        "_main/egb_fleet_adapter/config/fleet_config.yaml"
    )

    for label, path in [
        ("spawner", spawner),
        ("fleet_service_bridge", service_bridge),
        ("domain_bridge_node", domain_bridge_node),
        ("fleet config", template_cfg),
    ]:
        if not path or not os.path.exists(path):
            sys.exit(f"ERROR: rlocation lookup failed for {label} (got: {path!r})")

    fleet_adapter_node = os.environ.get("EGB_FLEET_ADAPTER_NODE")
    if not fleet_adapter_node:
        workspace = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
        if workspace:
            fleet_adapter_node = os.path.join(
                workspace, "bazel-bin", "egb_fleet_adapter", "fleet_adapter_node"
            )
    if not fleet_adapter_node or not os.path.exists(fleet_adapter_node):
        print(
            "[launch_all] WARNING: fleet_adapter_node not found at "
            f"{fleet_adapter_node!r}. Spawner will fail when starting a fleet. "
            "Build it with: bazel build //egb_fleet_adapter:fleet_adapter_node",
            file=sys.stderr,
        )

    clock_fd, clock_cfg = tempfile.mkstemp(
        prefix="domain_bridge_config_", suffix=".yaml"
    )
    with os.fdopen(clock_fd, "w") as f:
        f.write(
            f"from_domain: {foreign_domain}\n"
            f"to_domain: {ros_domain}\n"
            "topics:\n"
            "  /clock:\n"
            "    type: rosgraph_msgs/msg/Clock\n"
        )
    print(f"[launch_all] Generated /clock bridge config: {clock_cfg}")
    print(
        f"[launch_all] Bridging /clock from FOREIGN_DOMAIN_ID={foreign_domain} "
        f"to ROS_DOMAIN_ID={ros_domain}"
    )

    # Spawner invokes fleet_adapter_node per fleet; pass the resolved path
    # through the environment (may be missing if the user hasn't built it
    # yet — the spawner itself will surface a clearer error then).
    env = dict(os.environ)
    if fleet_adapter_node:
        env["EGB_FLEET_ADAPTER_NODE"] = fleet_adapter_node

    children: list[subprocess.Popen] = []

    def cleanup(*_):
        for p in children:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
        try:
            os.unlink(clock_cfg)
        except FileNotFoundError:
            pass

    signal.signal(signal.SIGINT, lambda *_: (cleanup(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (cleanup(), sys.exit(143)))

    try:
        children.append(
            subprocess.Popen(
                [spawner, "--ros-args", "-p", f"template_config_file:={template_cfg}"],
                env=env,
                preexec_fn=os.setsid,
            )
        )
        children.append(
            subprocess.Popen(
                [
                    service_bridge,
                    "--ros-args",
                    "--log-level",
                    "rmw_cyclonedds_cpp:=error",
                ],
                env=env,
                preexec_fn=os.setsid,
            )
        )
        children.append(
            subprocess.Popen(
                [
                    domain_bridge_node,
                    clock_cfg,
                    "--ros-args",
                    "--log-level",
                    "rmw_cyclonedds_cpp:=error",
                ],
                env=env,
                preexec_fn=os.setsid,
            )
        )

        # Exit as soon as any child does, with that child's status.
        pid, status = os.wait()
        rc = os.WEXITSTATUS(status) if os.WIFEXITED(status) else 1
        return rc
    finally:
        cleanup()


if __name__ == "__main__":
    sys.exit(main())
