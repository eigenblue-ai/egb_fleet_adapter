# Third-party licenses

This product fetches or links against the components below. Full license
texts are available at the linked URLs and in each component's own
source tree (downloaded into the Bazel cache at build time).

## Fetched at build time via Bazel Central Registry (`bazel_dep`)

| Component                                         | Version      | License                                                      |
| ------------------------------------------------- | ------------ | ------------------------------------------------------------ |
| rules_cc                                          | 0.1.1        | Apache-2.0                                                   |
| rules_python                                      | 1.1.0        | Apache-2.0                                                   |
| rules_rust                                        | 0.70.0       | Apache-2.0                                                   |
| rules_foreign_cc                                  | 0.13.0       | Apache-2.0                                                   |
| rules_shell                                       | 0.3.0        | Apache-2.0                                                   |
| platforms                                         | 0.0.10       | Apache-2.0                                                   |
| yaml-cpp                                          | 0.8.0        | MIT                                                          |
| nlohmann_json                                     | 3.11.3.bcr.1 | MIT                                                          |
| curl                                              | 8.4.0.bcr.1  | curl (MIT-like)                                              |
| eigen                                             | 3.4.0.bcr.3  | MPL-2.0 (with BSD/LGPL parts gated off by default)           |
| boost.asio / boost.circular_buffer / boost.system | 1.83.0.bcr.1 | Boost Software License 1.0                                   |
| zlib                                              | 1.3.1.bcr.3  | zlib                                                         |
| tinyxml2                                          | 10.0.0       | zlib                                                         |
| zstd                                              | 1.5.7        | BSD-3-Clause or GPL-2.0 (dual-licensed; this build uses BSD) |
| zenoh-c                                           | 1.7.2.bcr.1  | Apache-2.0 or EPL-2.0                                        |
| zenoh-cpp                                         | 1.7.2.bcr.1  | Apache-2.0 or EPL-2.0                                        |
| proj                                              | 9.8.0        | X/MIT                                                        |
| libuuid (util-linux)                              | 2.41.2       | BSD-3-Clause                                                 |
| ccd (libccd)                                      | 2.1.0.bcr.1  | BSD-3-Clause                                                 |

## Fetched at build time via `http_archive` (extensions/non_module_deps.bzl)

### Open-RMF (release-jazzy-240617 coordinated release)

| Component                      | Version | License                                                                                 |
| ------------------------------ | ------- | --------------------------------------------------------------------------------------- |
| open-rmf/rmf_internal_msgs     | 3.3.1   | Apache-2.0                                                                              |
| open-rmf/rmf_api_msgs          | 0.3.1   | Apache-2.0                                                                              |
| open-rmf/rmf_building_map_msgs | 1.4.1   | Apache-2.0                                                                              |
| open-rmf/rmf_utils             | 1.6.2   | Apache-2.0                                                                              |
| open-rmf/rmf_battery           | 0.3.1   | Apache-2.0                                                                              |
| open-rmf/rmf_task              | 2.5.1   | Apache-2.0                                                                              |
| open-rmf/rmf_traffic           | 3.3.3   | Apache-2.0 (bundles FCL 0.6 — BSD-3-Clause)                                             |
| open-rmf/rmf_ros2              | 2.7.2   | Apache-2.0 (provides rmf_fleet_adapter, rmf_traffic_ros2, rmf_task_ros2, rmf_websocket) |

### Other native deps

| Component                      | License      |
| ------------------------------ | ------------ |
| ros2/domain_bridge             | Apache-2.0   |
| ros/pluginlib                  | BSD-3-Clause |
| ros/class_loader               | BSD-3-Clause |
| pboettch/json-schema-validator | MIT          |
| zaphoyd/websocketpp            | BSD-3-Clause |

## Patches applied to fetched sources

Listed under `extensions/*.patch`. Each patch carries a short description
of why the modification is needed; the patched files are not redistributed
in source form in this repository.

## Apache-2.0 components

Apache-2.0 requires the upstream NOTICE files to be preserved. They are
present in each fetched archive's own `NOTICE`/`LICENSE` files within the
Bazel external cache at build time.
