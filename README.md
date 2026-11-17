# UR30 Inspection Workcell (ROS 2 Jazzy)

A learning workspace for **MoveIt 2**, motion planning and **ros2_control** on a Universal Robots **UR30**
with a **Zivid 2+ MR60** 3D camera on the tool flange. Gazebo Harmonic and MuJoCo are both installed
as simulator backends. See [PLAN.md](PLAN.md) for the full plan and milestones.

| Milestone | Status |
|---|---|
| **M1** Devcontainer + UR30 + Zivid + MoveIt on mock hardware | ✅ this commit |
| M2 Controller configs + `moveit_py` examples | ⏳ |
| M3 Gazebo factory (Warehouse, conveyor, parts, fence) + simulated Zivid point cloud | ⏳ |
| M4 MuJoCo backend | ⏳ |
| M5 Moving conveyor | ⏳ |

## Host prerequisites

- Linux with X11, Docker, VS Code and the **Dev Containers** extension (`ms-vscode-remote.remote-containers`)
- `/dev/dri` for GPU rendering (Intel/AMD with Mesa). No NVIDIA toolkit is needed.
- The `render` group GID in `.devcontainer/devcontainer.json` (`--group-add=110`) must match the host:
  `getent group render`
- X11 access for local containers is granted automatically by `initializeCommand`
  (`xhost +SI:localuser:$USER`).

## Getting started

1. Open this folder in VS Code and run **Dev Containers: Reopen in Container**.
   The first build takes about 10–20 minutes and produces an image of about 10 GB.
2. `post-create.sh` runs automatically. It fetches `zivid_description` (`dependencies.repos` → `src/external/`),
   installs rosdeps, runs `colcon build --symlink-install`, and checks the GPU renderer.
3. In a new terminal inside the container:

   ```bash
   ros2 launch ur_inspection_bringup moveit_mock.launch.py          # UR30 + Zivid, MoveIt + RViz
   ```

   In RViz, drag the interactive marker, then click **Plan & Execute** in the MotionPlanning panel.

4. Optionally, run the automated Milestone 1 checks in a second terminal while the launch is running:

   ```bash
   ros2 run ur_inspection_bringup verify_m1
   ```

## Workspace layout

```
.devcontainer/                Dockerfile, devcontainer.json, post-create.sh
dependencies.repos            source deps (zivid-ros, pinned) -> src/external/ (git-ignored)
src/
  ur_inspection_description/  UR xacro (reuses ur_robot_driver's) + Zivid bracket/camera on tool0
  ur_inspection_moveit_config/ SRDF (reuses UR's macro + camera collision exemptions), MoveIt configs, move_group launch
  ur_inspection_bringup/      moveit_mock.launch.py, verify_m1
```

### Useful commands

```bash
colcon build --symlink-install && source install/setup.bash
ros2 launch ur_inspection_bringup moveit_mock.launch.py ur_type:=ur10e   # any UR model works
ros2 run tf2_ros tf2_echo tool0 zivid_optical_frame
xacro src/ur_inspection_description/urdf/ur_inspection.urdf.xacro ur_type:=ur30 name:=ur30 > /tmp/ur.urdf
ros2 run rqt_controller_manager rqt_controller_manager
gz sim shapes.sdf                                                       # Gazebo Harmonic GUI check
python3 -m mujoco.viewer                                                # MuJoCo viewer check
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot open display` / RViz will not start | On the host run `xhost +SI:localuser:$USER`, and check that `echo $DISPLAY` in the container matches the host |
| Black windows, or `glxinfo -B` shows `llvmpipe` | `/dev/dri` is not passed through, or the render GID is wrong. Check `ls -l /dev/dri` and `id` inside the container |
| `Could not find package zivid_description` | Re-run `bash .devcontainer/post-create.sh` (needs network access for `vcs import`) |
| Topics from another machine or session show up | Set a different `ROS_DOMAIN_ID` on the host before opening the container |
