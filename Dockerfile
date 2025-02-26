# syntax=docker/dockerfile:1.13.0
ARG DEV_FROM_STAGE=builder
ARG EXPORT_FROM_STAGE=compiler
ARG OVERLAY_WS=/opt/mtr_ws
ARG SHIP_FROM_STAGE=runner
ARG WS_CACHE_ID=mtr

ARG FROM_IMAGE=base
# Stage from full image tag name for dependabot detection
FROM ubuntu:noble-20250127 AS base

################################################################################
# MARK: baser - setup base image using snapshots
################################################################################
FROM $FROM_IMAGE AS baser

# install setup packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
  && rm -rf /var/lib/apt/lists/*

# configure ubuntu snapshot
RUN UBUNTU_DEB_SNAPSHOT=$(date -r /var/lib/dpkg/info +%Y%m%dT%H%M%SZ) && \
    echo "APT::Snapshot \"${UBUNTU_DEB_SNAPSHOT}\";" \
    | tee /etc/apt/apt.conf.d/50snapshot

# configure ROS distro
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_VERSION=2

# configure ROS snapshot
ARG ROS_DEB_SNAPSHOT=2025-01-20
ARG ROS_SOURCE_URL=http://snapshots.ros.org/${ROS_DISTRO}/${ROS_DEB_SNAPSHOT}/ubuntu
RUN . /etc/os-release && \
    apt-key adv \
      --keyserver hkp://keyserver.ubuntu.com:80 \
      --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
    echo "deb ${ROS_SOURCE_URL} ${UBUNTU_CODENAME} main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# edit apt config for caching
RUN mv /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache && \
    # Given fixed snapshots, just cache apt update once
    apt-get update && echo v1

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# setup timezone and locales
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get install -y --no-install-recommends \
      locales \
      tzdata

# configure workspace
ARG OVERLAY_WS
ENV OVERLAY_WS=$OVERLAY_WS
WORKDIR $OVERLAY_WS

# install bootstrap tools
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
      gettext-base \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      wget \
      zstd

# init and update rosdep
ENV ROS_HOME=/opt/.ros
RUN rosdep init && \
    rosdep update \
      --rosdistro $ROS_DISTRO

# add default user for devcontainer
ENV DEV_USER=ubuntu
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

################################################################################
# MARK: cacher - cache source and dependency instructions
################################################################################
FROM baser AS cacher

# copy overlay source
COPY ./ ./src/microntracker-ros

# remove skiped packages
RUN find ./ \
      -name "COLCON_IGNORE" \
      | xargs dirname | xargs rm -rf || true && \
    # remove unnecessary packages
    colcon list --paths-only \
      --packages-skip-up-to  \
        microntracker_ros \
      | xargs rm -rf && \
    # check remaining packages
    colcon list --paths-only \
      --packages-up-to  \
        microntracker_ros \
      | tee /tmp/packages.txt && \
    [ -s /tmp/packages.txt ] || exit 1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN dep_types=(\
      "exec:--dependency-types=exec" \
      "test:--dependency-types=test" \
      "build:"\
    ) && \
    for dep_type in "${dep_types[@]}"; do \
      IFS=":"; set -- $dep_type; \
      rosdep install -y \
        --from-paths src \
        --ignore-src \
        # --skip-keys " \
        #     example_package_name \
        #   "\
        --reinstall \
        --simulate \
        ${2} \
        | grep 'apt-get install' \
        | awk -F' ' '{print $4}' | sed "s/'//g" \
        | sort > /tmp/${1}_debs.txt; \
    done

################################################################################
# MARK: runner - setup runtime dependencies for deployment
################################################################################
FROM baser AS runner

# install packages for field work
COPY docker/runner_apt_debs.txt /tmp/runner_apt_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    cut -d# -f1 < /tmp/runner_apt_debs.txt | envsubst \
      | xargs apt-get install -y --no-install-recommends

COPY --from=cacher /tmp/exec_debs.txt /tmp/exec_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/exec_debs.txt xargs apt-get install -y --no-install-recommends

################################################################################
# MARK: tester - setup test dependencies for validation
################################################################################
FROM runner AS tester

# install bootstrap tools
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
      git \
      git-lfs \
      python3-colcon-clean \
      python3-colcon-common-extensions \
      python3-colcon-metadata \
      python3-colcon-mixin \
      python3-pip \
    && pip3 install --break-system-packages \
      git+https://github.com/ruffsl/colcon-cache.git@6076815bbb574da028d270cf6eb93bdd5b29c7f4
ENV COLCON_EXTENSION_BLOCKLIST="colcon_core.package_augmentation.cache_git"

# setup default colcon configuration
ENV COLCON_HOME=/opt/.colcon
RUN ln -s $OVERLAY_WS/src/microntracker-ros/docker/.colcon /opt/.colcon

# install packages for test work
COPY docker/tester_apt_debs.txt /tmp/tester_apt_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
  cut -d# -f1 < /tmp/tester_apt_debs.txt | envsubst \
  | xargs apt-get install -y --no-install-recommends

COPY --from=cacher /tmp/test_debs.txt /tmp/test_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/test_debs.txt xargs apt-get install -y --no-install-recommends

# Override default flake8 configuration for black compatibility
COPY docker/.colcon/flake8.ini \
  /opt/ros/jazzy/lib/python3.12/site-packages/ament_flake8/configuration/ament_flake8.ini

################################################################################
# MARK: builder - setup build dependencies for compilation
################################################################################
FROM tester AS builder

COPY --from=cacher /tmp/build_debs.txt /tmp/
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    < /tmp/build_debs.txt xargs apt-get install -y --no-install-recommends

# install packages for build work
COPY docker/builder_apt_debs.txt /tmp/builder_apt_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
  cut -d# -f1 < /tmp/builder_apt_debs.txt | envsubst \
    | xargs apt-get install -y --no-install-recommends

# Symlink for docker outside of docker
RUN ln -s /var/run/docker-host.sock /var/run/docker.sock

# install awscli
RUN --mount=type=cache,sharing=locked,target=/tmp/aws \
  cd /tmp/aws && \
  wget -nc \
    https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip && \
  unzip -q /tmp/aws/awscli-exe-linux-x86_64.zip && \
  ./aws/install && \
  rm -rf ./aws

# setup default ccache configuration
ENV CCACHE_DIR="$OVERLAY_WS/.ccache"
ENV CCACHE_TEMPDIR="$CCACHE_DIR/tmp"
ENV CCACHE_CONFIGPATH="$OVERLAY_WS/src/microntracker-ros/docker/.ccache/ccache.conf"

# capture environment to modify layer digest
RUN env > /tmp/env.txt

################################################################################
# MARK: dever - setup user account for development
################################################################################
FROM $DEV_FROM_STAGE AS dever

# install packages for development work
COPY docker/dever_apt_debs.txt /tmp/dever_apt_debs.txt
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
  cut -d# -f1 < /tmp/dever_apt_debs.txt | envsubst \
  | xargs apt-get install -y --no-install-recommends

################################################################################
# MARK: seeder - seed workspace artifacts for caching
################################################################################
FROM alpine AS seeder
ARG OVERLAY_WS
ARG WS_CACHE_ID

ARG CLEAR_WS_CACHE
RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    if [ -n "$CLEAR_WS_CACHE" ]; then \
      echo "Clearing cache!" && \
      rm -rf $OVERLAY_WS/* && \
      echo "Cache cleared!"; \
    fi

ARG SEED_WS_CACHE
RUN --mount=type=bind,source=./,target=/seeder/\
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    if [ -n "$SEED_WS_CACHE" ]; then \
      echo "Seeding cache!" && \
      cp -rT /seeder/cache/$OVERLAY_WS $OVERLAY_WS && \
      echo "Cache seeded!"; \
    fi

RUN echo $(date) > /tmp/seeder_stamp.txt

################################################################################
# MARK: compiler - compile workspace artifacts for deployment
################################################################################
FROM builder AS compiler
ARG WS_CACHE_ID

# build overlay source
ARG BUST_BUILD_CACHE
RUN --mount=type=cache,from=cacher,target=/cacher \
    --mount=type=cache,from=seeder,target=/seeder \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    ln -s /cacher/$OVERLAY_WS/src src && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon cache lock \
      --dirhash-reset && \
    colcon clean packages -y \
      --packages-select-cache-invalid \
      --packages-select-cache-key build \
      --base-select install && \
    colcon build \
      --packages-skip-cache-valid \
      --packages-up-to  \
        microntracker_ros \
      || true && \
    echo $(date) > /tmp/compiler_stamp.txt

ARG FAIL_ON_BUILD_FAILURE=1
RUN --mount=type=cache,from=cacher,target=/cacher \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    BUILD_FAILED=$( \
      colcon list \
        --packages-select-build-failed) && \
    if [ -n "$BUILD_FAILED" ]; then \
      echo "BUILD_FAILED: \n$BUILD_FAILED" && \
      ([ -z "$FAIL_ON_BUILD_FAILURE" ] || exit 1); \
    fi

################################################################################
# MARK: validator - validate workspace artifacts for development
################################################################################
FROM tester AS validator
ARG WS_CACHE_ID

# test overlay build
ARG BUST_TEST_CACHE
RUN --mount=type=cache,from=cacher,target=/cacher \
    --mount=type=cache,from=compiler,target=/compiler \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . install/setup.sh && \
    colcon test \
      --packages-skip-cache-valid \
      --packages-up-to  \
        microntracker_ros \
    && echo $(date) > /tmp/validator_stamp.txt

ARG FAIL_ON_TEST_FAILURE=1
RUN --mount=type=cache,from=cacher,target=/cacher \
    --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS \
    . install/setup.sh && \
    colcon test-result \
      --verbose \
      || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1)

################################################################################
# MARK: dancer - multi-stage for caches dancing
################################################################################
FROM $EXPORT_FROM_STAGE AS dancer
ARG WS_CACHE_ID

RUN --mount=type=cache,id=$WS_CACHE_ID,sharing=private,target=$OVERLAY_WS,readonly \
    echo "Exporting cache!" && \
    mkdir -p /dancer/$OVERLAY_WS && \
    cp -rT $OVERLAY_WS /dancer/$OVERLAY_WS && \
    echo "Cache exported!"

################################################################################
# MARK: exporter - multi-stage for exporting caches
################################################################################
FROM $EXPORT_FROM_STAGE AS exporter

COPY --link --from=dancer /dancer/$OVERLAY_WS $OVERLAY_WS

################################################################################
# MARK: shipper - setup production images using shared instructions
################################################################################
FROM $SHIP_FROM_STAGE AS shipper

# Automatically source the production ROS install
RUN echo 'source "$OVERLAY_WS/install/setup.bash"' >> /root/.bashrc

################################################################################
# MARK: debugger - stage target for debuggin in production
################################################################################
FROM shipper AS debugger

COPY --link --from=dancer /dancer/$OVERLAY_WS/build $OVERLAY_WS/build
COPY --link --from=dancer /dancer/$OVERLAY_WS/install $OVERLAY_WS/install

################################################################################
# MARK: releaser - stage target for releasing in production
################################################################################
FROM shipper AS releaser

COPY --link --from=dancer /dancer/$OVERLAY_WS/build $OVERLAY_WS/build
COPY --link --from=dancer /dancer/$OVERLAY_WS/install $OVERLAY_WS/install
