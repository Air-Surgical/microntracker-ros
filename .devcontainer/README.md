# Dev Containers Basics

For info on using devcontainers with ROS, check the Nav2 docs here:

- https://navigation.ros.org/development_guides/devcontainer_docs/index.html


For more on the devcontainer specification, check the official docs here:

- https://containers.dev/
- https://containers.dev/implementors/json_reference/#lifecycle-scripts

## Lifecycle Scripts

When first opening this superproject from a devcontainer, the `initializeCommand`, defined in the devcontainer config file `.devcontainer/devcontainer.json`, is invoked to build the base image `mtc:devcontainer` for launching the docker container. Specifically, this initiation step uses `docker buildx` to bake the desired target defined in the docker bake file in the root of the superproject. By default, the target for development is used, but can be modified to point to any target defined in the bake file.

Once the base image is built, the devcontainer tooling will extend the base image further with any additional features or extensions defined in the devcontainer config file, as well as launch the container using the runtime settings defined in the same config. Once the container is up, several more commands are used to initialize the shell environment, arrange filesystem permissions for the designated user `ubuntu`, and finally kick off the compilation of the colcon overlay workspace.
