# Plan: ROS 2 Jazzy UR30 Inspection Workcell Devcontainer (MoveIt 2 + Gazebo Harmonic + MuJoCo)

> **Confidence Score: 94%** — Every requirement and scope decision is answered, and the upstream launch interfaces, package names and extension IDs have been checked against their sources. The remaining uncertainty is technical and can only be settled by building: whether Intel iGPU rendering inside the container keeps up with a full-resolution (2448 × 2048) simulated Zivid, and how much tuning a UR30 converted to MuJoCo's format (MJCF) needs.

## Problem Statement
The project directory `~/Documents/Workspace/Udemy/ROS` is empty. There is no reproducible environment for learning MoveIt 2, motion planning and ros2_control on a Universal Robots arm. The long-term target is an inspection workcell: a **UR30** on a pedestal next to a conveyor with primitive workpieces, inside a safety fence in the Gazebo **Warehouse**, with a **Zivid 2+ MR60** 3D camera on the tool flange. Either Gazebo Harmonic or MuJoCo can serve as the physics backend. **Milestone 1 ("done") means** a VS Code devcontainer on **ROS 2 Jazzy** (Ubuntu 24.04) that builds from a clean local git checkout and has MoveIt 2, Gazebo Harmonic, MuJoCo, `mujoco_ros2_control` and the UR packages installed, with GUI and GPU working. One launch command must bring up RViz with the MoveIt 2 MotionPlanning panel on the **UR30 with the Zivid MR60 mounted on the flange**. Plan-and-execute must work against mock hardware, and MoveIt must treat the camera as part of the robot's collision model. Later milestones add the Gazebo factory, the MuJoCo backend and a moving conveyor.

### Host facts already verified
| Item | Value | Implication |
|---|---|---|
| Host OS | Pop!_OS 22.04 (Jammy) | Irrelevant to the distro; the container runs Ubuntu 24.04 / Jazzy |
| Docker / Compose | 29.6.2 / v5.3.1 | Ready |
| VS Code + Dev Containers | `ms-vscode-remote.remote-containers` **already installed on the host** | VS Code "Reopen in Container" is the entry point. Nothing to install on the host |
| GPU | Intel integrated (Arc / Meteor Lake class), **no NVIDIA** | Pass `/dev/dri` through and use Mesa. No `nvidia-container-toolkit` |
| Display | X11 (`DISPLAY=:1`) | Mount `/tmp/.X11-unix`, plus `xhost +SI:localuser:$USER` on the host |
| CPU / RAM / Disk | 22 cores / 30 GB / 85 GB free | Enough. The image is about 8–12 GB |

### Upstream facts verified for the chosen stack
| Component | Finding | Consequence |
|---|---|---|
| ROS 2 **Jazzy** | Ubuntu 24.04 Noble, EOL May 2029, pairs with **Gazebo Harmonic** | Base image `osrf/ros:jazzy-desktop-full` |
| `ur_description` (jazzy) | Has `config/ur30/` | UR30 kinematics, limits and meshes are available through apt |
| `ur_robot_driver/ur_control.launch.py` (jazzy) | Accepts `description_launchfile:=<abs path>`, which must start a `robot_state_publisher` that publishes `/robot_description`. Also accepts `use_mock_hardware`, `controllers_file` and `launch_rviz` | The custom UR30 + Zivid URDF plugs into the stock control launch with no fork |
| `ur_moveit_config/ur_moveit.launch.py` (jazzy) | Accepts `ur_type:=ur30`, but **hardcodes** `MoveItConfigsBuilder(package_name="ur_moveit_config")` and its own `srdf/ur.srdf.xacro`. `move_group` waits for the `/robot_description` topic | The stock SRDF has no collision exemptions for the bracket or camera. M1 therefore needs its own `ur_inspection_moveit_config` whose SRDF reuses UR's `ur_macro.srdf.xacro` and adds the camera pairs |
| `ur_simulation_gz` (jazzy) | apt, v2.5.x, takes `ur_type` | Reference for the Gazebo backend (M3) |
| `mujoco_ros2_control` | apt: `ros-jazzy-mujoco-ros2-control`, v0.0.x, tested on Jazzy + MuJoCo 3.4 | Install from apt. Young package, so expect API changes |
| `zivid-ros` `master` | ROS 2 (`ament_cmake`, v3.4.0). `zivid_description/urdf/macros/zivid_camera.xacro` takes `prefix`, `model:=ZIVID_2_PLUS_MR60` and `field_of_view`, with the optical-center offset (0.049, 0.03202, 0.0295) m and a 2.5° optical tilt | Build **only `zivid_description`** from source. `zivid_camera` and `zivid_samples` need the Zivid SDK plus OpenCL and get `COLCON_IGNORE` |
| Zivid 2+ MR60 specs | Focus 600 mm, typical range 350–900 mm (recommended 300–1100 mm), FOV about 570 × 460 mm at 600 mm, **2448 × 2048** | Simulated sensor at full resolution: hFOV ≈ 50.8°, clip 0.3–1.1 m, low default rate (2 Hz); resolution and rate are xacro args |
| Gazebo Fuel | `OpenRobotics/Warehouse` exists ("simple warehouse walls") | World includes the Fuel URI; cache `~/.gz/fuel` in a volume |
| MuJoCo Menagerie | UR5e and UR10e only, **no UR30** | UR30 MJCF is generated from `ur_description` (M4) |
| VS Code Marketplace | All recommended extension IDs below confirmed to exist | No placeholder IDs left |

## Options

### Option 1: Swappable Simulator Backends (one ros2_control stack, pick a sim per launch) — ✅ SELECTED
A single robot description, `ur_inspection.urdf.xacro`, drives one ros2_control configuration. It wraps `ur_description`'s UR30 macro and attaches a mounting bracket plus the `zivid_description` MR60 macro to `tool0`. A `sim:=mock|gazebo|mujoco` argument selects the hardware plugin: `mock_components/GenericSystem`, `gz_ros2_control/GazeboSimSystem` or `mujoco_ros2_control`. In **M1** the description reaches the stock UR control launch through `description_launchfile`, and MoveIt runs from `ur_inspection_moveit_config`. That package reuses UR's SRDF macro and kinematics, adds the bracket and camera collision exemptions and a `zivid_optical_frame`, and gets its own `move_group` and RViz launch, so the camera on the flange is part of planning from day one. Controllers are the stock `ros2_controllers` set (JTC, forward position/velocity, admittance), tuned through YAML (controls scope A). All project code is **Python** (`ament_python` packages, `rclpy`, `moveit_py`). The **Gazebo** backend (M3) loads `factory.sdf`. It contains the OpenRobotics Warehouse, a UR30 on a 0.8 m pedestal, a conveyor about 1.0 m from the base at about 0.9 m height, primitive workpieces (boxes and cylinders), and a safety fence. The MR60 renders as a full-resolution `rgbd_camera` bridged to ROS through `ros_gz_bridge`. The **MuJoCo** backend (M4) uses a UR30 MJCF plus a simplified primitive scene: conveyor, workpieces and fence, with no warehouse building. The same layout values come from one shared YAML file. Milestones: **M1** devcontainer + UR30 + Zivid + MoveIt (mock) → **M2** controller configs and `moveit_py` examples → **M3** Gazebo factory + simulated Zivid point cloud → **M4** MuJoCo backend → **M5** moving conveyor.

**Pros:**
- This is the standard industry pattern. MoveIt configuration and Python nodes are written once and run on mock, Gazebo, MuJoCo, and later a real UR30 through `ur_robot_driver`.
- No forks of upstream UR packages. The custom description plugs into `ur_control.launch.py` through its supported `description_launchfile` argument.
- The camera is part of collision checking from M1, so later inspection paths plan against the real tool geometry.
- Every backend uses an upstream ros2_control integration, so there is no custom bridge code.

**Cons:**
- M1 is bigger than "stock packages only": two custom packages (description and MoveIt config) are needed before the first successful launch.
- The scene is described twice (SDF and simplified MJCF). This is reduced by keeping the layout in one shared YAML file.
- The simulated MR60 is an ideal depth camera. Full resolution on an Intel iGPU may run slowly in Gazebo (see Risks).
- The UR30 needs a URDF-to-MJCF conversion with hand-tuned actuators, because Menagerie has no UR30.

### Option 2: Co-Simulation (MuJoCo is the authoritative physics, Gazebo renders the world and sensors) — ❌ RULED OUT
You chose swappable backends. Co-simulation needs a custom MuJoCo→Gazebo joint-state bridge with clock sync, and it splits contacts across two engines. That is too much maintenance for a learning project.

**Pros:**
- One factory world with Gazebo sensor rendering and MuJoCo arm dynamics at the same time.

**Cons:**
- No upstream support. Custom bridge, clock sync and split contacts.
- Sync jitter would misalign camera data and joint states, which hurts inspection work.

### Option 3: Gazebo-First, MuJoCo Installed but Deferred — ❌ RULED OUT as an architecture (its ordering is kept)
You chose swappable backends from the start. The useful part, building and validating Gazebo before MuJoCo, is kept as the milestone order inside Option 1 (M3 before M4). MuJoCo and `mujoco_ros2_control` are installed in the image from M1.

**Pros:**
- Shortest path to a working Gazebo inspection cell. This benefit is kept through the milestone order.

**Cons:**
- MuJoCo would not be a first-class backend, which conflicts with your stated architecture.

## Clarifying Questions
<!-- To annotate offline: place **ANSWER** your response on the line immediately after a question. To add a new question, add a line **QUESTION** your question anywhere in this section. Then run /review-plan to apply. -->

All clarifying questions have been answered.

## Recommended VS Code Extensions (approved — all are installed in the container)

These go in `devcontainer.json → customizations.vscode.extensions`, and VS Code installs them automatically inside the container on first open. Dev Containers is the only host-side extension, and it is **already installed** on your host. All IDs were confirmed in the VS Code Marketplace.

| Extension | ID | Why |
|---|---|---|
| Dev Containers *(host only, already installed)* | `ms-vscode-remote.remote-containers` | Runs the whole workflow |
| Python | `ms-python.python` | Python language support |
| Pylance | `ms-python.vscode-pylance` | Type checking and IntelliSense for `rclpy` and `moveit_py` |
| Python Debugger | `ms-python.debugpy` | Debugging ROS Python nodes |
| Ruff | `charliermarsh.ruff` | Linter and formatter |
| Robot Developer Extensions for ROS 2 | `Ranch-Hand-Robotics.rde-ros-2` | Successor to the deprecated `ms-iot.vscode-ros`: ROS commands and launch debugging |
| XML | `redhat.vscode-xml` | URDF, xacro, SDF, SRDF, package.xml, MJCF |
| YAML | `redhat.vscode-yaml` | ros2_control and MoveIt YAML |
| URDF Visualizer | `morningfrog.urdf-visualizer` | Quick URDF/xacro preview |
| Container Tools | `ms-azuretools.vscode-containers` | Dockerfile editing and container inspection (successor to `vscode-docker`) |
| CMake | `twxs.cmake` | `CMakeLists.txt` syntax (`zivid_description`, `ament_cmake` config packages) |
| C/C++ | `ms-vscode.cpptools` | Reading upstream C++ (ros2_control, MoveIt) |
| Git Graph | `mhutchie.git-graph` | Visual git history |
| markdownlint | `DavidAnson.vscode-markdownlint` | README and PLAN hygiene |
| Code Spell Checker | `streetsidesoftware.code-spell-checker` | Typos in docs and comments |

## Change Tracker

| File / Component | Change Type | Estimated Scope | Milestone |
|-----------------|-------------|-----------------|-----------|
| `.git/` (`git init`, local only, no remote), `.gitignore` (`build/ install/ log/`), `.dockerignore` | add | small | M1 |
| `.devcontainer/Dockerfile` — `osrf/ros:jazzy-desktop-full`; apt: `ros-jazzy-moveit`, `ros-jazzy-moveit-py`, `ros-jazzy-ur`, `ros-jazzy-ur-simulation-gz`, `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-mujoco-ros2-control`, `ros-jazzy-rqt-controller-manager`, `ros-jazzy-rqt-joint-trajectory-controller`, `ros-jazzy-plotjuggler-ros`, `ros-dev-tools`, `mesa-utils`, `python3-pip`; pip `mujoco` (version matched to `mujoco_ros2_control`); non-root user UID/GID 1000 in `video` and `render` | add | medium | M1 |
| `.devcontainer/devcontainer.json` — `runArgs` (`--device=/dev/dri`, `--network=host`, `--ipc=host`, `--group-add` render GID), X11 socket mount, `DISPLAY`, `ROS_DOMAIN_ID`, named volumes for `build/ install/ log/` and `~/.gz/fuel`, the 14 container extensions above, `postCreateCommand` | add | small | M1 |
| `.devcontainer/post-create.sh` — `vcs import src < dependencies.repos`, `COLCON_IGNORE` in `zivid_camera` and `zivid_samples`, `rosdep install`, `colcon build --symlink-install`, `.bashrc` sourcing, `glxinfo -B` GPU check | add | small | M1 |
| `dependencies.repos` — `zivid/zivid-ros` pinned to a commit (only `zivid_description` is built) | add | small | M1 |
| `src/ur_inspection_description/` (`ament_cmake`, data only) — `urdf/ur_inspection.urdf.xacro` (UR30 macro + `zivid_mount.xacro` on `tool0`; args `ur_type`, `sim`), `urdf/zivid_mount.xacro` (bracket link, `zivid_camera` macro with `ZIVID_2_PLUS_MR60`, Gazebo `rgbd_camera` tags gated on `sim==gazebo`, xacro args for resolution and rate defaulting to 2448 × 2048 at 2 Hz), `urdf/ur_inspection.ros2_control.xacro`, `launch/rsp.launch.py` (the `description_launchfile` target) | add | medium | **M1** |
| `src/ur_inspection_moveit_config/` (`ament_cmake`) — `srdf/ur_inspection.srdf.xacro` (includes `ur_moveit_config/srdf/ur_macro.srdf.xacro` + `disable_collisions` for `wrist_3_link`/`tool0`/bracket/`zivid_*`), `config/kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml`, `pilz_cartesian_limits.yaml`, `moveit_controllers.yaml` (copied from or referencing `ur_moveit_config`), `launch/move_group.launch.py` (`MoveItConfigsBuilder(package_name="ur_inspection_moveit_config")` + `wait_for_robot_description`), `rviz/moveit.rviz` | add | medium | **M1** |
| `src/ur_inspection_bringup/` (`ament_python`) — `launch/moveit_mock.launch.py`: `ur_control.launch.py ur_type:=ur30 use_mock_hardware:=true launch_rviz:=false description_launchfile:=<ur_inspection_description>/launch/rsp.launch.py` + `ur_inspection_moveit_config/move_group.launch.py` | add | small | **M1 success test** |
| `README.md` — host prerequisite (`xhost +SI:localuser:$USER`), Reopen in Container, M1 launch command, troubleshooting (black windows, `llvmpipe`, DRI) | add | small | M1 |
| `src/ur_inspection_bringup/config/ur30_controllers.yaml` — JTC, forward position/velocity, admittance | add | small | M2 |
| `src/ur_inspection_bringup/launch/bringup.launch.py` — unified `sim:=mock\|gazebo\|mujoco`, `use_sim_time` wiring | add | small | M2–M4 |
| `src/ur_inspection_examples/` (`ament_python`) — `moveit_py` scripts: named poses, Cartesian scan path over the conveyor at 600 mm standoff, capture and save a point cloud | add | medium | M2–M3 |
| `src/ur_inspection_bringup/config/workcell_layout.yaml` — single source for pedestal, conveyor and part poses and fence extents (read by the SDF and MJCF generators and by the MoveIt planning scene) | add | small | M3 |
| `src/ur_inspection_gazebo/` (`ament_python`) — `worlds/factory.sdf.xacro` (Fuel `OpenRobotics/Warehouse`, pedestal, static conveyor, primitive workpieces, fence), `launch/sim_gz.launch.py`, `config/gz_bridge.yaml` (clock, Zivid image, depth and points) | add | medium–large | M3 |
| `src/ur_inspection_mujoco/` (`ament_python`) — `mjcf/ur30.xml` (converted; DAE→OBJ meshes; actuators and gains), `mjcf/scene.xml` (primitive conveyor, parts, fence, MR60 camera), `scripts/urdf_to_mjcf.py`, `launch/sim_mujoco.launch.py` | add | large | M4 |
| Conveyor motion — Gazebo conveyor/track plugin, or a Python node that moves the workpieces | add | medium | M5 (follow-on) |

*Change types: add / modify / delete / refactor. Scope: small (< 50 lines), medium (50–200 lines), large (200+ lines).*

### Milestone 1 acceptance test
1. VS Code **Dev Containers: Reopen in Container** builds and runs `post-create.sh` with no errors, and the 14 extensions are present in the container.
2. `ros2 pkg list` includes `moveit_ros_planning_interface`, `moveit_py`, `ur_description`, `ur_moveit_config`, `ur_simulation_gz`, `ros_gz_sim`, `gz_ros2_control`, `mujoco_ros2_control`, `zivid_description`, `ur_inspection_description`, `ur_inspection_moveit_config` and `ur_inspection_bringup`. `python3 -c "import mujoco"` succeeds.
3. `glxinfo -B` reports the Intel Mesa renderer (not `llvmpipe`), and `gz sim shapes.sdf` opens a GUI window.
4. `ros2 launch ur_inspection_bringup moveit_mock.launch.py` opens RViz with the **UR30 and the Zivid MR60 on the flange**. `ros2 run tf2_ros tf2_echo tool0 zivid_optical_frame` returns a transform. Plan & Execute to a random valid goal succeeds, and MoveIt reports no self-collision at the home pose.
5. A planning request that pushes the camera into `upper_arm_link` is **rejected** for collision, which proves the camera is in the collision model.
6. `git log` shows the initial commit, and `git status` is clean (no `build/ install/ log/`).

### Milestone 1 status — ✅ complete (2026-11-17)
Verified in a freshly created container:
- 4 packages built: `zivid_description`, `ur_inspection_description`, `ur_inspection_moveit_config`, `ur_inspection_bringup`.
- `glxinfo -B` reports "Mesa Intel(R) Arc(tm) Graphics (MTL)", so rendering is hardware-accelerated. RViz reports OpenGL 4.6, and `gz sim shapes.sdf` runs.
- `ros2 run ur_inspection_bringup verify_m1` passes all checks:
  - `tool0` → `zivid_optical_frame` = (-0.089, 0.057, 0.103) m
  - the home state is collision-free
  - a camera-to-arm contact is detected (`shoulder_link` ↔ `zivid_base_link`)
  - OMPL plan, and execution on mock hardware to within 0.0093 rad
- MuJoCo 3.12.0 (Python) matches `ros-jazzy-mujoco-vendor`, and `moveit_py` imports.

Deviations from the plan above:
- `build/ install/ log/` live in the bind-mounted repo (git-ignored) instead of named volumes. This is simpler and survives container rebuilds. Only the Fuel cache (`~/.gz`) uses a named volume.
- The Zivid Gazebo `rgbd_camera` sensor tags and the resolution/rate xacro args move to **M3**, where they can be tested. M1's `zivid_mount.xacro` contains the bracket and camera geometry only.
- Added `ros-jazzy-ros2controlcli` (for `ros2 control list_controllers`) and a `verify_m1` acceptance script.
- `mujoco_ros2_control` is now at v0.1.4 on apt, not v0.0.x.
- The render-group GID (110) is hard-coded in `devcontainer.json` `runArgs` for this host.
- `rsp.launch.py` forwards UR arguments to `ur_rsp.launch.py` explicitly, and `moveit_mock.launch.py` scopes the UR include in a `GroupAction`. Both fix launch-argument scoping problems found during testing.

## Impact & Risks

### Risks
- **Full-resolution Zivid on the Intel iGPU (medium likelihood, medium severity, M3):** 2448 × 2048 RGB-D is about 5 M points per frame. ogre2 depth rendering through Mesa may fall below real time and slow the physics. Mitigation: full resolution stays the default as requested, at a low default rate (2 Hz) and with sensor `always_on` false or triggered capture. Resolution and rate are xacro args, so a reduced setting is a one-flag change.
- **Intel GPU rendering in the container (medium likelihood, medium severity, M1):** A missing `/dev/dri` or `render` group gives black windows or `llvmpipe`. Mitigation: pass the host render GID through `--group-add`, and check `glxinfo -B` in post-create and in the acceptance test.
- **Custom SRDF diverging from UR's (low likelihood, medium severity, M1):** If upstream changes `ur_macro.srdf.xacro`, the overlay could break. Mitigation: include the upstream macro rather than copying it, and pin it through the apt package version.
- **Zivid mesh and frame names (low likelihood, low severity, M1):** The exact link names generated by `zivid_camera.xacro` (`<prefix>…`) are confirmed during implementation, and the SRDF pairs and `tf2_echo` target are adjusted to match.
- **UR30 → MJCF conversion (high likelihood, medium severity, M4):** No Menagerie UR30 exists, DAE meshes need converting, and actuator gains need tuning. Mitigation: a scripted conversion, Menagerie's UR10e as a structural template, and joint names identical to `ur_description`.
- **`mujoco_ros2_control` maturity (medium likelihood, medium severity, M4):** v0.0.x. Mitigation: pin the apt version and match the MuJoCo pip version.
- **Fuel download (low likelihood, low severity, M3):** The Warehouse model needs network access on first run. Mitigation: a persistent `~/.gz/fuel` volume.
- **Image size and build time (high likelihood, low severity):** About 8–12 GB and a 10–20 minute first build, on a host disk that is 81% used. Mitigation: `--no-install-recommends` and apt list cleanup.
- **X11 security (low likelihood, low severity):** Mitigation: `xhost +SI:localuser:$USER`, documented.

### New Issues That May Arise
- High-rate full-resolution point clouds can saturate DDS. Use best-effort sensor QoS, a low rate, or Cyclone DDS.
- `use_sim_time` must be consistent across MoveIt, controllers and bridges once Gazebo or MuJoCo owns the clock (M3+).
- The SDF and MJCF scenes can drift apart. `workcell_layout.yaml` is the single source of truth.
- The ideal simulated MR60 will not reproduce real Zivid artifacts (reflections, HDR, structured-light shadows), so inspection algorithms will need retuning on real data.
- Pop!_OS Mesa or kernel updates may occasionally break DRI passthrough until the container is rebuilt.
- Running ROS on the host and in the container at the same time with the same `ROS_DOMAIN_ID` causes topic crosstalk.
- Without a git remote, the repository has no off-machine backup until you add one.
