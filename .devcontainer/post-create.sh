#!/usr/bin/env bash
# Runs once inside the container after it is created (devcontainer postCreateCommand).
# Fetches source dependencies, installs rosdeps, builds the workspace and checks GPU rendering.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

# ROS setup scripts reference unset variables, so relax -u while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

echo "==> Importing source dependencies (dependencies.repos -> src/external)"
mkdir -p src/external
vcs import --skip-existing --input dependencies.repos src/external

# zivid-ros: only zivid_description is needed. The others need the proprietary Zivid SDK / a real camera.
for pkg in zivid_camera zivid_interfaces zivid_rviz_plugin zivid_samples; do
  if [ -d "src/external/zivid-ros/${pkg}" ]; then
    touch "src/external/zivid-ros/${pkg}/COLCON_IGNORE"
  fi
done

echo "==> Installing rosdep dependencies"
sudo apt-get update -qq
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

echo "==> Building workspace"
colcon build --symlink-install

echo "==> GPU / OpenGL check"
if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  renderer="$(glxinfo -B 2>/dev/null | grep -E 'OpenGL renderer string' || true)"
  echo "    ${renderer:-<glxinfo could not reach the X server>}"
  if echo "${renderer}" | grep -qi llvmpipe; then
    echo "    WARNING: software rendering (llvmpipe). Check /dev/dri passthrough and the render group GID in devcontainer.json."
  fi
else
  echo "    Skipped (no DISPLAY or glxinfo)."
fi

echo "==> Done. Open a new terminal (or 'source install/setup.bash') and run:"
echo "    ros2 launch ur_inspection_bringup moveit_mock.launch.py"
