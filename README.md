# EgbFleetAdapter

A standalone, fully bazelized [Open-RMF](https://github.com/open-rmf)
fleet adapter for AMR fleets. Builds without `apt`, `rosdep`, or `colcon`
— every dependency (RMF C++ stack, `domain_bridge`, `pluginlib`,
`zenoh-c`/`zenoh-cpp`) is fetched and compiled from source via Bazel
modules and `http_archive`.

## Layout

```
egb_fleet_msgs/      # ROS2 interface package (msg/srv/action)
egb_fleet_adapter/   # RMF fleet adapter (C++ lib + main + 3 plugins)
egb_fleet_spawner/   # Python service node + cross-domain bridge
extensions/          # http_archives + upstream patches
third_party/         # BUILD files authored for each external source
```

## Build

```bash
bazel build //egb_fleet_adapter:fleet_adapter_node
bazel build //egb_fleet_spawner:launch_all
```

## Run

```bash
FOREIGN_DOMAIN_ID=45 ROS_DOMAIN_ID=41 \
  bazel run //egb_fleet_spawner:launch_all
```

This brings up:

1. `egb_fleet_spawner` — Python service node advertising
   `/egb_fleet/start_fleet_adapters` and `/egb_fleet/stop_fleet_adapters`
2. `fleet_service_bridge` — C++ bridge that mirrors those services
   across DDS domains via `domain_bridge`
3. `domain_bridge_node` — `/clock` topic bridge between the two domains

Calling `/egb_fleet/start_fleet_adapters` spawns one
`fleet_adapter_node` per fleet, each connecting to the RMF API server
over websocket.

## License

Apache 2.0. See `LICENSE`, `NOTICE`, and `THIRD_PARTY_LICENSES.md`.
