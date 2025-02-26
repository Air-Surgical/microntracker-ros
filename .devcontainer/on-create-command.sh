#!/usr/bin/env bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

# Generate locales for git shell
sudo locale-gen $LANG

# Set git config for submodules
git config --local include.path ../.devcontainer/config/.gitconfig

# Set git config for colcon cache
git config --global --add safe.directory "*"

# Install pre-commit hooks via isolated environment
pre-commit install --overwrite --allow-missing-config
# Patch pre-commit hook for use only with devcontainer
sed -i '1i\
#!/usr/bin/env bash\
if [ -z "$DEVCONTAINER" ]; then\
  exit 0\
fi\
source /opt/ros/$ROS_DISTRO/setup.bash\
' .git/hooks/pre-commit
# Symlink all submodule to use superproject hooks
find .git/modules -type d -path '*/hooks' \
    -exec sh -c 'rm -rf "$1" || true && ln -sf ../../hooks "$1"' _ {} \;

# NOTE: This is slow if not using a mounted volumes,
# i.e. using workspace from the docker image directly,
# presumably due to docker overlayfs driver overhead.
# If needing to use workspace pre-baked into base image,
# consider using a new volume to be auto populated with
# the workspace pre-baked in image via devcontainer tools.
# Either by deleting old volume from the docker engine
# Or simpler changing volume name in devcontainer.json
sudo chown -R :ubuntu $OVERLAY_WS
# Recursively update group permissions for workspace
# to allow write access via dev users current group
sudo chmod -R g+rwx $OVERLAY_WS

# Recursively update group permissions for ros home
# to allow write access such as ros node logs
sudo chown -R :ubuntu /opt/.ros
sudo chmod -R g+rwx /opt/.ros

# Recursively update permissions for user home
# to allow write access such as config dotfiles
sudo chown -R $(id -u):$(id -g) $HOME
