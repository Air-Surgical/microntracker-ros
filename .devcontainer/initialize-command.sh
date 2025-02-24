#!/usr/bin/env bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Use first argument as target name
target=$1

# Intialize directories for personalized bind mounts
mkdir -p $HOME/mtchome

################################################################################
# MARK: Build image - bake image using local Dockerfile for local dev container
# unset DEV_FROM_STAGE
################################################################################

# Check opt out ENV for docker rebuild
if [ -n "$MTR_DOCKER_REBUILD_OPT_OUT" ]; then
    echo "Opting out of docker rebuild"
    exit 0
fi

# Bake the target and export locally to static tag
docker buildx bake --load \
    --file docker-bake.hcl \
    --set $target.tags=mtc:devcontainer \
    $target
