# Requires dist/ staged first (see build-image.yml).

FROM ubuntu:24.04 AS runtime

# For the /usr/bin/env python3 shebang in the staged scripts.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates \
        libatomic1 \
        libstdc++6 \
        libuuid1 \
        libssl3 \
        zlib1g \
        python3 \
        tini \
    && rm -rf /var/lib/apt/lists/*

COPY dist/launch_all                  /opt/egb_fleet/launch_all
COPY dist/launch_all.runfiles         /opt/egb_fleet/launch_all.runfiles
# Resolved at runtime via EGB_FLEET_ADAPTER_NODE, so staged on its own.
COPY dist/fleet_adapter_node          /opt/egb_fleet/fleet_adapter_node
COPY dist/fleet_adapter_node.runfiles /opt/egb_fleet/fleet_adapter_node.runfiles
RUN chmod +x /opt/egb_fleet/launch_all /opt/egb_fleet/fleet_adapter_node

ENV EGB_FLEET_ADAPTER_NODE=/opt/egb_fleet/fleet_adapter_node \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launchers reference their impls relative to the runfiles _main dir.
WORKDIR /opt/egb_fleet/launch_all.runfiles/_main
ENTRYPOINT ["/usr/bin/tini", "--", "/opt/egb_fleet/launch_all"]
