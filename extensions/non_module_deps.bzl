"""Module extension declaring http_archive repositories for non-bzlmod deps.

Land progressively across phases:
  Phase 2 — RMF interface (msg) repos: rmf_internal_msgs, rmf_building_map_msgs
  Phase 3 — leaf C++ deps: rmf_utils (vendored FCL 0.6 lives inside rmf_traffic),
            nlohmann_json_schema_validator
  Phase 4 — heavy C++ libs: rmf_traffic, rmf_battery, rmf_task, rmf_ros2
  Phase 5 — pluginlib stack: pluginlib, class_loader (if missing from rules_ros2)
  Phase 6 — domain_bridge
  Phase 7 — zenoh: zenoh_c, zenoh_cpp
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _non_module_deps_impl(_ctx):
    # ---- Phase 2: RMF interface (msg) packages ----
    # Sha256 was captured at first download and pinned for hermeticity.
    # All RMF packages tracked against open-rmf's release-jazzy-240617
    # coordinated release — the last officially-tested-together set
    # (rmf.repos in the open-rmf/rmf repo).
    http_archive(
        name = "rmf_internal_msgs",
        build_file = "//third_party/rmf:rmf_internal_msgs.BUILD.bazel",
        sha256 = "cac276fcd8bb7964ab345b6dca2ec5bc3517bc33e39de1ba983878c2e06043f7",
        strip_prefix = "rmf_internal_msgs-3.3.1",
        urls = ["https://github.com/open-rmf/rmf_internal_msgs/archive/refs/tags/3.3.1.tar.gz"],
    )

    # Same version/sha used by tools/imrmf-map-editor — keep in sync.
    http_archive(
        name = "rmf_building_map_msgs",
        build_file = "//third_party/rmf:rmf_building_map_msgs.BUILD.bazel",
        sha256 = "846dfe1351546594a15d64c9c6a50ce94cdf9fd09e612a2b4608c6fa7d47a02f",
        strip_prefix = "rmf_building_map_msgs-1.4.1",
        urls = ["https://github.com/open-rmf/rmf_building_map_msgs/archive/refs/tags/1.4.1.tar.gz"],
    )

    # ---- Phase 3: Leaf C++ utility libs ----
    # rmf_utils is mostly headers (impl_ptr, clone_ptr, optional, math,
    # AssignID, Modular, RateLimiter); the single .cpp is compiled into the
    # cc_library target.
    http_archive(
        name = "rmf_utils",
        build_file = "//third_party/rmf:rmf_utils.BUILD.bazel",
        sha256 = "4b7dd0a70753db6f2a9fe887ccc94eed84e84a7e41c49587497bcf2a60ff2bc8",
        strip_prefix = "rmf_utils-1.6.2",
        urls = ["https://github.com/open-rmf/rmf_utils/archive/refs/tags/1.6.2.tar.gz"],
    )

    # websocketpp 0.8.2 + asio 1.28 (header-only). Required by rmf_websocket.
    # BCR has both, but BCR's websocketpp 0.8.2 pulls boost.asio 1.87+ which
    # removed `expires_from_now()` that websocketpp itself still calls
    # upstream — known incompat. So we keep the local http_archives.
    http_archive(
        name = "websocketpp",
        build_file_content = """
cc_library(
    name = "websocketpp",
    hdrs = glob(["websocketpp/**/*.hpp"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
        sha256 = "6ce889d85ecdc2d8fa07408d6787e7352510750daa66b5ad44aacb47bea76755",
        strip_prefix = "websocketpp-0.8.2",
        urls = ["https://github.com/zaphoyd/websocketpp/archive/0.8.2.tar.gz"],
    )
    http_archive(
        name = "asio",
        build_file_content = """
cc_library(
    name = "asio",
    hdrs = glob(["include/**/*.hpp", "include/**/*.ipp"]),
    defines = ["ASIO_STANDALONE"],
    includes = ["include"],
    visibility = ["//visibility:public"],
)
""",
        sha256 = "226438b0798099ad2a202563a83571ce06dd13b570d8fded4840dbc1f97fa328",
        strip_prefix = "asio-asio-1-28-0/asio",
        urls = ["https://github.com/chriskohlhoff/asio/archive/asio-1-28-0.tar.gz"],
    )

    # ---- Phase 4: RMF C++ libs + their non-RMF deps ----
    # libccd (collision detection primitives) is required by FCL — comes
    # from BCR as @ccd//:ccd (see bazel_dep in MODULE.bazel).

    # rmf_traffic bundles vendored FCL 0.6 inside thirdparty/fcl/. Its
    # BUILD.bazel exposes both `:fcl` and `:rmf_traffic` as cc_library targets.
    http_archive(
        name = "rmf_traffic",
        build_file = "//third_party/rmf:rmf_traffic.BUILD.bazel",
        sha256 = "0292c4a1f83a5196aaa84bfad8001af62b87dce3baccfb7d6c5bd7c38b6a3d32",
        strip_prefix = "rmf_traffic-3.3.3",
        urls = ["https://github.com/open-rmf/rmf_traffic/archive/refs/tags/3.3.3.tar.gz"],
    )

    http_archive(
        name = "rmf_battery",
        build_file = "//third_party/rmf:rmf_battery.BUILD.bazel",
        sha256 = "abbf3ac48d7e0bc2ceed170d38c3eaff4e2fa568350b6c84088816677e0b5493",
        strip_prefix = "rmf_battery-0.3.1",
        urls = ["https://github.com/open-rmf/rmf_battery/archive/refs/tags/0.3.1.tar.gz"],
    )

    # rmf_api_msgs — C++ INTERFACE library; schema headers generated from
    # .json files via genrule mirroring upstream CMake.
    # NOTE: the /archive/refs/tags/ URL returns 404 intermittently; the direct
    # codeload URL it redirects to is more reliable for bazel's fetcher.
    http_archive(
        name = "rmf_api_msgs",
        build_file = "//third_party/rmf:rmf_api_msgs.BUILD.bazel",
        sha256 = "8f392dcd3b181240870328b802dd8d80c4dc0440e9642360ea3bc09dc51c4016",
        strip_prefix = "rmf_api_msgs-0.3.1",
        type = "tar.gz",
        urls = [
            "https://codeload.github.com/open-rmf/rmf_api_msgs/tar.gz/refs/tags/0.3.1",
            "https://github.com/open-rmf/rmf_api_msgs/archive/refs/tags/0.3.1.tar.gz",
        ],
    )

    # nlohmann/json-schema-validator — required by rmf_task_sequence and
    # rmf_task_ros2 / rmf_fleet_adapter for runtime schema validation.
    http_archive(
        name = "nlohmann_json_schema_validator",
        build_file = "//third_party/rmf:nlohmann_json_schema_validator.BUILD.bazel",
        sha256 = "24cbb114609cc9b43d4018b8d03e082ff5d2f26f5dce8bd36538097267b63af9",
        strip_prefix = "json-schema-validator-2.4.0",
        urls = ["https://github.com/pboettch/json-schema-validator/archive/refs/tags/2.4.0.tar.gz"],
    )

    http_archive(
        name = "rmf_task",
        build_file = "//third_party/rmf:rmf_task.BUILD.bazel",
        sha256 = "56c94c1ddab1e80e8a2c89e07393857ae051dcc72415a40531eda59809da575e",
        strip_prefix = "rmf_task-2.5.1",
        urls = ["https://github.com/open-rmf/rmf_task/archive/refs/tags/2.5.1.tar.gz"],
    )

    # rmf_ros2 bundles rmf_websocket, rmf_traffic_ros2, rmf_task_ros2,
    # rmf_fleet_adapter (+ a few rmf_*_node packages we don't expose).
    http_archive(
        name = "rmf_ros2",
        build_file = "//third_party/rmf:rmf_ros2.BUILD.bazel",
        sha256 = "18619e83f739167572b2f1c113eecd089db863da1a082e2bcaee61e803094f81",
        strip_prefix = "rmf_ros2-2.7.2",
        urls = ["https://github.com/open-rmf/rmf_ros2/archive/refs/tags/2.7.2.tar.gz"],
    )

    # ---- Phase 5: pluginlib (class_loader is already in rules_ros2) ----
    http_archive(
        name = "pluginlib",
        build_file = "//third_party/pluginlib:pluginlib.BUILD.bazel",
        sha256 = "ff358633291e08fe245720f71f8d5338d3314cb6053e5a737261f9ea496072a3",
        strip_prefix = "pluginlib-5.5.0",
        urls = ["https://github.com/ros/pluginlib/archive/refs/tags/5.5.0.tar.gz"],
    )

    # ---- Phase 6: domain_bridge ----
    http_archive(
        name = "domain_bridge",
        build_file = "//third_party:domain_bridge.BUILD.bazel",
        sha256 = "fcc9b8b11a1e0411d615f4bc7c5a07ee4d6481a7058e08b2bc0462be02a2f8b7",
        strip_prefix = "domain_bridge-0.5.0",
        urls = ["https://github.com/ros2/domain_bridge/archive/refs/tags/0.5.0.tar.gz"],
    )

    # ---- Phase 7: zenoh ----
    # zenoh-c + zenoh-cpp come from the Bazel Central Registry — the
    # `zenoh-c` 1.7.2.bcr.1 module wires up rules_rust + crate_universe and
    # a custom cbindgen rule, and exposes a top-level cc_library. See
    # bazel_dep(name = "zenoh-c") + bazel_dep(name = "zenoh-cpp") in
    # MODULE.bazel.
    pass  # nothing to declare locally for zenoh anymore.

non_module_deps = module_extension(implementation = _non_module_deps_impl)
