#!/usr/bin/env bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Check opt out ENV for colcon rebuild
if [ -n "$MTR_COLCON_REBUILD_OPT_OUT" ]; then
    echo "Opting out of colcon rebuild"
    exit 0
fi

cd $OVERLAY_WS

colcon cache lock

BUILD_PACKAGES=$(
    colcon list \
        --names-only \
        --packages-up-to \
            microntracker_ros \
        --packages-skip-cache-valid \
        --packages-select-cache-key \
            build \
    | xargs)
echo BUILD_PACKAGES: $BUILD_PACKAGES

# DEBUG: Uncoment for more sterile but slower builds
# colcon clean packages --yes \
#     --packages-select ${BUILD_PACKAGES} \
#     --base-select build install

. /opt/ros/$ROS_DISTRO/setup.sh
colcon build \
    --packages-select \
        $BUILD_PACKAGES \
    || true
