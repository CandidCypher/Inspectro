# ROS 2 Jazzy (Ubuntu 24.04 Noble) dev image for the UR30 + Zivid inspection workcell.
# Includes MoveIt 2, the Universal Robots stack, Gazebo Harmonic (ros_gz), ros2_control,
# and MuJoCo (mujoco_ros2_control + Python bindings).
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Must match the MuJoCo version vendored by ros-jazzy-mujoco-vendor (pulled in by mujoco_ros2_control).
ARG MUJOCO_PY_VERSION=3.12.0

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ROS packages + dev tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-moveit-py \
        ros-jazzy-ur \
        ros-jazzy-ur-simulation-gz \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-mujoco-ros2-control \
        ros-jazzy-rqt-controller-manager \
        ros-jazzy-rqt-joint-trajectory-controller \
        ros-jazzy-plotjuggler-ros \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-dev-tools \
        mesa-utils \
        python3-pip \
        sudo \
        git \
        bash-completion \
        less \
        nano \
        gdb \
    && rm -rf /var/lib/apt/lists/*

# `ros2 control ...` CLI (list/switch controllers, inspect hardware interfaces)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo Python bindings (viewer, MJCF experiments). Ubuntu 24.04 marks the system Python as
# externally managed, so this is an explicit, pinned system-wide install.
RUN pip3 install --no-cache-dir --break-system-packages "mujoco==${MUJOCO_PY_VERSION}"

# Non-root user matching the host UID/GID. The Noble base image ships an "ubuntu" user on UID 1000,
# so remove it first.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --create-home --shell /bin/bash --uid "${USER_UID}" --gid "${USER_GID}" \
        --groups video,sudo "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    # Pre-create the Gazebo Fuel cache dir so the named volume mounted there is user-owned.
    && mkdir -p "/home/${USERNAME}/.gz" \
    && chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}/.gz"

# Shell setup: ROS underlay, workspace overlay (if built), colcon helpers.
RUN { \
      echo ''; \
      echo '# --- ROS 2 Jazzy ---'; \
      echo 'source /opt/ros/jazzy/setup.bash'; \
      echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null || true'; \
      echo 'if [ -f "${WS_DIR:-/workspaces/ROS}/install/setup.bash" ]; then source "${WS_DIR:-/workspaces/ROS}/install/setup.bash"; fi'; \
    } >> "/home/${USERNAME}/.bashrc"

USER ${USERNAME}
WORKDIR /home/${USERNAME}
