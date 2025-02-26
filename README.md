# microntracker-ros
ROS Wrapper for ClaroNav MicronTracker

## Index

- [Dev Containers](.devcontainer/README.md) - Reproducible workstation environments

For more detailed build information, view the [docs](docs/README.md).

## Quick Start

To get started, follow the instructions below.

### Prerequisites

First, ensure your using a recent enough version of Docker Engine that supports [BuildKit](https://docs.docker.com/build/buildkit/). If you plan on running robot simulations locally, Hardware Acceleration for sensor raytracing and 3D rendering is also recommended. While other compatible devcontainer tools may be used, Visual Studio Code is recommended for simplicity.

#### System Software
- [Docker Engine](https://docs.docker.com/engine/install/)
  - https://get.docker.com - simple universal install script
  - [Linux post-installation](https://docs.docker.com/engine/install/linux-postinstall/) - manage Docker as a non-root user
- [Git LFS](https://git-lfs.github.com/) - for managing large assets
  - Use for version controlling media such as figures
  - Necessary for cloning example simulation files
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) - optional for enabling Hardware Acceleration
  - [Installing the Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) - only necessary host running Docker Engine

#### Development Tools
- [Visual Studio Code](https://code.visualstudio.com/) - alternative to Dev Containers CLI
  - [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) - via SSH, Tunnels, Containers, WSL
    - [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) - specific to just Containers
  - [Docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) - for introspecting Docker daemon
  - [Using SSH keys](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials#_using-ssh-keys) - sharing Git credentials with container
- [Dev Container CLI](https://github.com/devcontainers/cli) - alternative to VSCode
  - [Installation via NPM](https://github.com/devcontainers/cli?tab=readme-ov-file#npm-install) - for custom install setup
  - [Installation via VSCode](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli) - for simple install setup
    - Note: CLI installed via VSCode is warped but bugged, install via NPM is recommended for now
    - https://github.com/devcontainers/cli/issues/799

### Cloning, Building and Running

Next, recursively clone this repository and included submodules, bake default image tags using buildx, and then simply run containers using the build docker image.

```shell
# Clone the repository and submodules
git clone --recurse-submodules -j8 \
  git@github.com:Air-Surgical/microntracker-ros.git

# Change into the repository directory
cd microntracker-ros

# Bake the builder image tag as a test
docker buildx bake builder

# Run container from image as a test
docker run -it --rm mtc:builder bash
```

### Launching Development Containers

Finally, use the CLI to bring up and exec into the Dev Container:

```shell
# To bring up a dev container
devcontainer up --workspace-folder .
# Or to bring up without using previous dev container
devcontainer up --workspace-folder . --remove-existing-container
# Or to bring up without using previous build cache
devcontainer up --workspace-folder . --remove-existing-container --build-no-cache
# To exec into existing dev container
devcontainer exec --workspace-folder . bash
```

Alternatively, open VSCode and use the Remote Containers extension:

```shell
code .
# Press Ctrl+Shift+P to open the Command Palette
#  Type and select `Dev Containers: Reopen in Container`
# Or to bring up without using previous dev container
#  Type `Dev Containers: Rebuild and Reopen in Container`
# Or to bring up without using previous build cache
#  Type `Dev Containers: Rebuild Without Cache and Reopen in Container`
```

Note: using Dev Containers from a remote host is also possible:

-  [Open a folder on a remote SSH host in a container](https://code.visualstudio.com/docs/devcontainers/containers#_open-a-folder-on-a-remote-ssh-host-in-a-container)
-  [Open a folder on a remote Tunnel host in a container](https://code.visualstudio.com/docs/devcontainers/containers#_open-a-folder-on-a-remote-tunnel-host-in-a-container)

### Further Reading and Concepts

Afterwards, you may want to further familiarize yourself more with the following topics:

- Git Submodules
  - https://git-scm.com/book/en/Git-Tools-Submodules
  - https://git-scm.com/docs/git-submodule
- Docker
  - Multi-stage
    - https://docs.docker.com/build/building/multi-stage/
  - BuildKit
    - https://docs.docker.com/build/buildkit/
  - Bake
    - https://docs.docker.com/build/bake/
- Development Containers
  - https://navigation.ros.org/development_guides/devcontainer_docs/index.html
  - https://containers.dev/
  - https://code.visualstudio.com/docs/devcontainers/containers
