#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 The EgbFleetAdapter Authors

import json
import logging
import os
import signal
import subprocess
import sys
import tempfile
import threading
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from egb_fleet_msgs.srv import StartFleetAdapters, StopFleetAdapters


class FleetAdapterProcessManager:
    """Manages a single Fleet Adapter process"""

    def __init__(
        self,
        fleet_name: str,
        chargers: list,
        robot_name_pattern: str,
        initial_map: str,
        log_dir: str,
        template_config_file: str,
        debug: bool = False,
    ):
        self.fleet_name = fleet_name
        self.chargers = chargers
        self.robot_name_pattern = robot_name_pattern
        self.initial_map = initial_map
        self.debug = debug
        self.process = None
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Create log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = self.log_dir / f"{fleet_name}_{timestamp}.log"

        # Setup logging
        self.logger = logging.getLogger(f"fleet_{fleet_name}")
        self.logger.setLevel(logging.INFO)
        self.logger.handlers.clear()
        self.logger.propagate = False

        file_handler = logging.FileHandler(self.log_file)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        # Create dynamic config file with fleet name and chargers substitution
        self.config_file = self._create_fleet_config_file(template_config_file)

        self.stdout_thread = None
        self.stderr_thread = None

    def _create_fleet_config_file(self, template_file):
        """Create a temporary fleet config file with fleet_name, chargers, and robot_name_pattern substitution"""

        # Read the template file
        with open(template_file, "r") as f:
            config = yaml.safe_load(f)

        # Replace fleet name
        if "rmf_fleet" in config and "name" in config["rmf_fleet"]:
            config["rmf_fleet"]["name"] = self.fleet_name

        # Update robot_name_pattern with provided pattern
        if (
            self.robot_name_pattern
            and "rmf_fleet" in config
            and "dynamic_discovery" in config["rmf_fleet"]
        ):
            config["rmf_fleet"]["dynamic_discovery"][
                "robot_name_pattern"
            ] = self.robot_name_pattern

        # Replace charger in robot_template with first charger from list
        if self.chargers and len(self.chargers) > 0:
            if "rmf_fleet" in config and "robot_template" in config["rmf_fleet"]:
                config["rmf_fleet"]["robot_template"]["charger"] = self.chargers[0]

        # Update initial_map (floor) in robot_template
        if (
            self.initial_map
            and "rmf_fleet" in config
            and "robot_template" in config["rmf_fleet"]
        ):
            config["rmf_fleet"]["robot_template"]["initial_map"] = self.initial_map

        # TODO: If we need to handle multiple chargers per fleet, we could edit the free fleet adapter to support that
        # For now, using the first charger as the default for all robots in this fleet

        # Create temporary file
        temp_file = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".yaml",
            delete=False,
            prefix=f"fleet_config_{self.fleet_name}_",
        )
        yaml.dump(config, temp_file, default_flow_style=False)
        temp_file.close()

        self.logger.info(f"Created dynamic fleet config file: {temp_file.name}")
        self.logger.info(
            f"Fleet: {self.fleet_name}, Pattern: {self.robot_name_pattern}, Floor: {self.initial_map}, Chargers: {self.chargers}"
        )

        return temp_file.name

    def _stream_reader(self, stream, stream_name):
        """Read stream output and log it"""
        try:
            for line in iter(stream.readline, ""):
                if line:
                    line = line.strip()
                    if stream_name == "stdout":
                        self.logger.info(f"[{self.fleet_name}] {line}")
                    else:
                        self.logger.error(f"[{self.fleet_name}] {line}")
        except Exception as e:
            self.logger.error(f"Error reading {stream_name}: {e}")
        finally:
            stream.close()

    def start(self, server_uri: str, use_sim_time: bool = False):
        """Start Fleet Adapter process"""
        if self.process and self.process.poll() is None:
            self.logger.warning(
                f"Fleet adapter for {self.fleet_name} is already running"
            )
            return True

        try:
            self.logger.info(f"Starting fleet adapter for {self.fleet_name}")
            self.logger.info(f"Using config file: {self.config_file}")
            self.logger.info(f"Server URI: {server_uri}")

            # Path resolved by launch_all.py via rlocation; the
            # ament_index fallback is for non-bazel invocations.
            node_exe = os.environ.get("EGB_FLEET_ADAPTER_NODE")
            if not node_exe:
                from ament_index_python.packages import get_package_prefix

                node_exe = os.path.join(
                    get_package_prefix("egb_fleet_adapter"),
                    "lib",
                    "egb_fleet_adapter",
                    "fleet_adapter_node",
                )

            cmd = [
                node_exe,
                self.config_file,
                "--ros-args",
                "-r",
                "__node:=fleet_adapter_node",
                "-p",
                f"server_uri:={server_uri}",
            ]
            if self.debug:
                cmd = [
                    "gdb",
                    "-batch",
                    "-ex",
                    "run",
                    "-ex",
                    "bt full",
                    "-ex",
                    "info threads",
                    "-ex",
                    "thread apply all bt full",
                    "-ex",
                    "quit",
                    "--args",
                ] + cmd

            self.logger.info(f"Command: {' '.join(cmd)}")

            # The fleet_adapter_node sh launcher exec's a sibling impl via a
            # relative path and reads AMENT_PREFIX_PATH as a relative path
            # too — it expects cwd to be the binary's own runfiles root.
            # Also scrub the parent's RUNFILES_* env vars so the child
            # doesn't try to look itself up in launch_all's runfiles tree.
            popen_kwargs = {}
            if not self.debug and node_exe and os.path.exists(node_exe + ".runfiles"):
                popen_kwargs["cwd"] = os.path.join(node_exe + ".runfiles", "_main")
                popen_kwargs["env"] = {
                    k: v
                    for k, v in os.environ.items()
                    if k
                    not in (
                        "RUNFILES_DIR",
                        "RUNFILES_MANIFEST_FILE",
                        "RUNFILES_MANIFEST_ONLY",
                        "JAVA_RUNFILES",
                    )
                }

            # Start process with separate stdout/stderr and text mode
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,  # Create new process group
                **popen_kwargs,
            )

            # Start threads to read stdout/stderr
            self.stdout_thread = threading.Thread(
                target=self._stream_reader,
                args=(self.process.stdout, "stdout"),
                daemon=True,
            )
            self.stderr_thread = threading.Thread(
                target=self._stream_reader,
                args=(self.process.stderr, "stderr"),
                daemon=True,
            )

            self.stdout_thread.start()
            self.stderr_thread.start()

            self.logger.info(
                f"Fleet adapter started for {self.fleet_name} with PID {self.process.pid}"
            )

            # Give it a moment to start and check if it's still running
            import time

            time.sleep(2)
            if self.process.poll() is not None:
                self.logger.error(
                    f"Fleet adapter for {self.fleet_name} died immediately with exit code {self.process.returncode}"
                )
                return False

            self.logger.info(
                f"Fleet adapter for {self.fleet_name} started successfully and is running"
            )
            return True

        except Exception as e:
            self.logger.error(
                f"Failed to start fleet adapter for {self.fleet_name}: {e}"
            )
            import traceback

            self.logger.error(f"Traceback: {traceback.format_exc()}")
            return False

    def stop(self):
        """Stop Fleet Adapter process"""
        if not self.process:
            return True

        try:
            self.logger.info(f"Stopping fleet adapter for {self.fleet_name}")

            # Send SIGTERM to process group
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

            # Wait for process to terminate
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.logger.warning(
                    f"Fleet adapter for {self.fleet_name} didn't terminate, forcing kill"
                )
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=5)

            # Clean up temporary config file
            if os.path.exists(self.config_file):
                os.unlink(self.config_file)
                self.logger.info(f"Cleaned up config file: {self.config_file}")

            self.logger.info(f"Fleet adapter stopped for {self.fleet_name}")
            self.process = None
            return True

        except Exception as e:
            self.logger.error(
                f"Failed to stop fleet adapter for {self.fleet_name}: {e}"
            )
            return False

    def is_running(self):
        """Check if process is still running"""
        if not self.process:
            return False
        return self.process.poll() is None

    def get_log_file(self):
        """Get path to log file"""
        return str(self.log_file)


class FleetSpawner(Node):
    """Manages Fleet Adapter processes for multiple fleets"""

    def __init__(self):
        super().__init__("egb_fleet_spawner")

        # Default template config: EGB_FLEET_CONFIG env first, package share as fallback
        default_template_config = os.environ.get("EGB_FLEET_CONFIG", "")
        if not default_template_config:
            try:
                package_share_dir = get_package_share_directory("egb_fleet_adapter")
                default_template_config = os.path.join(
                    package_share_dir, "config", "fleet_config.yaml"
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Could not find package share directory: {e}"
                )

        # Declare parameters
        self.declare_parameter("log_directory", "/tmp/egb_fleet_spawner_logs")
        self.declare_parameter("template_config_file", default_template_config)
        self.declare_parameter("server_uri", "ws://0.0.0.0:8000/_internal")
        self.declare_parameter("debug", False)
        # Note: use_sim_time is automatically declared by rclpy, don't declare it again

        # Get parameters
        self.log_directory = self.get_parameter("log_directory").value
        self.template_config_file = self.get_parameter("template_config_file").value
        self.server_uri = self.get_parameter("server_uri").value
        self.debug = bool(self.get_parameter("debug").value)
        # Always use true for use_sim_time to sync with simulation clock
        self.use_sim_time = True

        # Set use_sim_time parameter on the node itself
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)]
        )

        # Validate required parameters
        if not self.template_config_file:
            self.get_logger().error("template_config_file parameter is required")
            raise ValueError("template_config_file not provided")

        if not os.path.exists(self.template_config_file):
            self.get_logger().error(
                f"Template config file not found: {self.template_config_file}"
            )
            raise FileNotFoundError(
                f"Template config file not found: {self.template_config_file}"
            )

        # Create log directory
        Path(self.log_directory).mkdir(parents=True, exist_ok=True)

        # Store active fleet adapters
        self.active_adapters = {}

        # Setup logging
        self.setup_logging()

        # Create services (plural - batch operations)
        self.create_service(
            StartFleetAdapters,
            "/egb_fleet/start_fleet_adapters",
            self.start_fleet_adapters_callback,
        )
        self.create_service(
            StopFleetAdapters,
            "/egb_fleet/stop_fleet_adapters",
            self.stop_fleet_adapters_callback,
        )

        # Create status publisher
        self.status_pub = self.create_publisher(
            String, "/egb_fleet/fleet_adapter_status", 10
        )
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f"Fleet Spawner initialized. Logs: {self.log_directory}")
        self.get_logger().info(f"Template config: {self.template_config_file}")
        self.get_logger().info(f"Server URI: {self.server_uri}")

        # Setup signal handlers for cleanup
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def setup_logging(self):
        """Setup main spawner logging"""
        log_file = Path(self.log_directory) / "egb_fleet_spawner.log"

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            handlers=[logging.FileHandler(log_file), logging.StreamHandler()],
        )

        self.logger = logging.getLogger("egb_fleet_spawner")

    def start_fleet_adapters_callback(self, request, response):
        """Start Fleet Adapters for multiple fleets (batch operation)"""
        try:
            # Initialize response arrays
            response.success = []
            response.messages = []

            # Process each adapter request
            for adapter_params in request.adapters:
                # Check if already running
                if adapter_params.fleet_name in self.active_adapters:
                    response.success.append(False)
                    response.messages.append(
                        f"Fleet adapter already running for {adapter_params.fleet_name}"
                    )
                    continue

                # Convert chargers from ROS message to Python list
                chargers = list(adapter_params.chargers)

                # Create and start process manager
                manager = FleetAdapterProcessManager(
                    adapter_params.fleet_name,
                    chargers,
                    adapter_params.robot_name_pattern,
                    adapter_params.initial_map,
                    self.log_directory,
                    self.template_config_file,
                    debug=self.debug,
                )

                if manager.start(self.server_uri, self.use_sim_time):
                    self.active_adapters[adapter_params.fleet_name] = manager
                    response.success.append(True)
                    response.messages.append(
                        f"Fleet adapter started for {adapter_params.fleet_name}"
                    )
                    self.get_logger().info(
                        f"Started fleet adapter: {adapter_params.fleet_name} (pattern: {adapter_params.robot_name_pattern}, floor: {adapter_params.initial_map}, chargers: {chargers})"
                    )
                else:
                    response.success.append(False)
                    response.messages.append(
                        f"Failed to start fleet adapter for {adapter_params.fleet_name}"
                    )

        except Exception as e:
            self.get_logger().error(f"Error starting fleet adapters: {e}")
            if not response.success:
                response.success = [False] * len(request.adapters)
                response.messages = [str(e)] * len(request.adapters)

        return response

    def stop_fleet_adapters_callback(self, request, response):
        """Stop Fleet Adapters for multiple fleets (batch operation)"""
        try:
            # Initialize response arrays
            response.success = []
            response.messages = []

            # Process each fleet name
            for fleet_name in request.fleet_names:
                if fleet_name not in self.active_adapters:
                    response.success.append(False)
                    response.messages.append(
                        f"Fleet adapter not found for {fleet_name}"
                    )
                    continue

                manager = self.active_adapters[fleet_name]
                if manager.stop():
                    del self.active_adapters[fleet_name]
                    response.success.append(True)
                    response.messages.append(f"Fleet adapter stopped for {fleet_name}")
                    self.get_logger().info(f"Stopped fleet adapter: {fleet_name}")
                else:
                    response.success.append(False)
                    response.messages.append(
                        f"Failed to stop fleet adapter for {fleet_name}"
                    )

        except Exception as e:
            self.get_logger().error(f"Error stopping fleet adapters: {e}")
            if not response.success:
                response.success = [False] * len(request.fleet_names)
                response.messages = [str(e)] * len(request.fleet_names)

        return response

    def publish_status(self):
        """Publish Fleet Adapter status"""
        try:
            status = {"active_adapters": len(self.active_adapters), "adapters": {}}

            # Check health and cleanup dead processes
            dead_adapters = []
            for fleet_name, manager in self.active_adapters.items():
                if manager.is_running():
                    status["adapters"][fleet_name] = {
                        "status": "running",
                        "log_file": manager.get_log_file(),
                    }
                else:
                    status["adapters"][fleet_name] = {
                        "status": "dead",
                        "log_file": manager.get_log_file(),
                    }
                    dead_adapters.append(fleet_name)

            for fleet_name in dead_adapters:
                self.get_logger().warning(
                    f"Fleet adapter for {fleet_name} died, cleaning up"
                )
                del self.active_adapters[fleet_name]

            # Publish status
            status_msg = String()
            status_msg.data = json.dumps(status, indent=2)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Cleanup all Fleet Adapters"""
        self.get_logger().info("Stopping all fleet adapters...")
        for fleet_name in list(self.active_adapters.keys()):
            manager = self.active_adapters[fleet_name]
            manager.stop()
            del self.active_adapters[fleet_name]
        self.get_logger().info("Cleanup complete")


def main(args=None):
    rclpy.init(args=args)

    try:
        spawner = FleetSpawner()
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    finally:
        if "spawner" in locals():
            spawner.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
