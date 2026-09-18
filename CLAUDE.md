# Project context for Steve (Neobotix MPO-700 + UR + NBV inspection)

> Snapshot date: 2026-09-18. This file is meant to be fully self-contained —
> anyone (or any agent) picking up this project should be able to work from
> this file alone, without needing access to any particular session's
> personal memory. Verify against current code/hardware before treating
> anything here as certain; things move fast in this project.

---

## User

- Mohamed Elbassiony (mohamed.elbassiony00@eng-st.cu.edu.eg), student.
- Working at University of Bonn, Humanoid Robots Lab (HRL), on a real robot
  ("Steve") there.
- **This is nominally a 4-person course project, but the user has done
  essentially all of the real-robot work alone.** Already escalated to
  teammates twice with no material change in their output. Plan as if the
  user is executing solo; treat any teammate contribution as a bonus, not
  a dependency.
- **Exam: 2026-09-25.** As of this snapshot that's 7 days away — the user
  is visibly time-pressured. Be direct/action-oriented, check things
  proactively (web search, reading code) rather than asking the user to
  chase down info, and don't pad responses.
- **Claude CAN run shell commands directly in this devcontainer** (bash,
  ip, sudo — passwordless, ros2, git, conda) via the Bash tool. This
  corrects an old assumption in earlier project notes that Claude couldn't
  exec here. Outbound HTTPS (e.g. `git clone` from GitHub) works.
- **Always put shell commands in their own fenced code block**, never
  inline in a sentence — explicit user preference, applies to every
  response with a command in it.
- The user runs persistent/interactive/GUI processes themselves in their
  own terminal (camera nodes, model servers, RViz, jogging scripts) —
  don't start these in the background without being asked, and don't
  intervene on an already-running user-owned process (resume, kill,
  restart) without asking first, even if it looks stuck/crashed. One-shot,
  blocking calls that return immediately (a single planning script, a
  single trajectory send-and-wait, a diagnostic query) are fine for Claude
  to run directly to verify its own work.

---

## Project goals — NBV Inspection Pipeline, UR5e on Steve

**Goal:** Next-Best-View (NBV) inspection loop on a UR arm — first built and
proven in PyBullet sim, now ported to the real robot ("Steve").

**Stack:** PyBullet (sim), CuRobo (GPU motion planning — the actual point of
the exercise, this is a Parallel Computing course project; CPU-vs-GPU
benchmarking is an explicit exam deliverable, so CuRobo is not swappable
for MoveIt2 as a shortcut), SAM3 (segmentation), classical PCA+ICP pose
estimation (see "Real object" below — replaced FoundationPose for this
specific object), custom CUDA/PyTorch ray-visibility kernels.

### Sim pipeline — DONE, ~92% coverage, not the current focus
Lives in a **separate repo**, `/home/ws/Parallel_Robotics_Lab` (cloned from
`github.com/MohamedElhadidy0019/Parallel_Robotics_Lab`, branches `main`/
`dev`). Six-stage architecture (A–F), see that repo's own files
(`nbv_planner.py`, `full_pipeline.py`, `nbv_core/`). Kept on CuRobo
`v0.7.8` (legacy API) deliberately, not the mainline "cuRoboV2" rewrite —
a documented breaking API change, not worth the risk this close to the
exam. Key gotchas already solved there (carried into the real-robot port,
don't rediscover):
- CuRobo solves everything relative to `base_link`, not world frame.
- Quaternion convention: sim/scipy use `xyzw`, CuRobo wants `wxyz`.
- `CollisionCheckerType.PRIMITIVE`, not the default MESH — MESH pulls in
  NVIDIA `warp`'s torch interop, which throws `AttributeError: module
  'warp' has no attribute 'torch'` on this hardware.
- Table collision proxy deliberately lowered a few cm below the true
  table height to avoid a permanent self-collision at rest pose.

Sim scene-design lessons (still valid if sim work resumes):
- Camera is on the wrist (`camera.get_cam_in_hand(...)`) — never propose a
  virtual-camera shortcut, explicitly rejected by the user once already.
- `table.urdf` has NO collision (commented out) — use a `GEOM_BOX`
  primitive with real collision for anything objects rest on.
- Use a THIN SLAB table, not a solid block — a solid block's side wall
  blocks the orbit camera.
- Orbit the robot-facing half of the object (`angles = linspace(pi, 2*pi,
  ...)`), not the far side (unreachable, off the table).
- Orbit radius must include the gripper's extension beyond the EEF link.
- Use `execute_joint_states` for motion (drives via physics), never
  `reset_robot` (teleports, bypasses physics).
- Don't seed `calculateInverseKinematics` with `currentPositions` — the
  joint ordering for that arg is unclear here and produced garbage
  configs; use fine incremental arc steps instead to keep IK on the same
  config branch.

### Sim setup, from scratch (conda-based, NOT uv — uv has no pip)
```bash
# 1. Clone shelf_gym into third_party/ (gitignored)
git clone --recurse-submodules -j8 \
  https://github.com/NilsDengler/manipulation_enhanced_map_prediction \
  third_party/shelf_gym_repo

# 2. Conda env (miniforge, Python 3.12)
conda create -n rob_env python=3.12
conda activate rob_env
conda install -c conda-forge "cgal<6"   # CGAL 6+ removed boost::optional, breaks skgeom

# 3. Apply patches
git apply patches/shelf_gym.patch --directory=third_party/shelf_gym_repo

# 4. skgeom deps in order
pip install "pybind11[global]==2.11.1"  # 2.12+ breaks skgeom def_property+keep_alive

# 5. Run repo install script FROM INSIDE the repo
cd third_party/shelf_gym_repo
bash install.sh
pip install -e . --no-build-isolation
cd ../..

# 6. Run
python nbv_env2.py
```
Known issues already solved (don't rediscover): CGAL must be `<6`
(conda-forge); pybind11 must be `==2.11.1`; `install.sh` must run from
INSIDE `third_party/shelf_gym_repo/`; `table.urdf` has no collision
(commented out) — objects fall through it, use a `GEOM_BOX` primitive
instead. Patches in `patches/shelf_gym.patch`: `setup.py` `find_packages`
fix, training deps commented out of `requirements.txt`,
`shelf_environment.py` `show_vis=False` (klampt Qt/GLUT crash),
`camera_utils.py` reshape/float32 fixes.

**Important**: the sim's original ~92%-coverage run was done on the user's
HOST machine, OUTSIDE this devcontainer — the setup above was never
actually run inside THIS container. The `rob_env` conda env that DOES exist
inside this devcontainer (referenced throughout the rest of this file) was
built fresh, separately, specifically for the real-robot CuRobo work, and
deliberately does NOT include the sim stack (PyBullet/shelf_gym/CGAL) —
"this spike doesn't need the sim scene, only CuRobo + ROS2." If sim work
ever resumes inside this container, setting up `rob_env` per the steps
above would COLLIDE with the existing real-robot CuRobo env of the same
name — use a different env name, or a genuinely fresh container/machine.

---

## Real robot — "Steve" (Neobotix MPO-700 + UR5e + no gripper)

### Hardware, confirmed from the LIVE running stack (not guessed)
- Base: Neobotix MPO-700 omnidirectional.
- Arm: **UR5e** (confirmed live — joint/TF prefix `ur5e`, active controller
  `/scaled_joint_trajectory_controller`, `robot_mode`/`safety_mode`
  topics present, `/ur_tool_comm` node running — all e-series-only
  signatures). An earlier pendant inspection suggested CB3/PolyScope 3.x;
  that was wrong for the actually-running bringup, don't resurrect it.
- **No gripper** on the real arm (unlike the sim's Robotiq 2F-85 config) —
  CuRobo's own stock `ur5e.yml` (gripper-less) is used directly for all
  real-robot planning, not an adapted sim config.
- Camera: Intel RealSense **D415**, mounted on the wrist (eye-in-hand),
  connected via USB — historically to the DEV PC directly for calibration
  work, not necessarily the onboard PC. The URDF also has an unrelated,
  never-used `wrist_d405_mount → wrist_camera_link` chain (CAD-guessed,
  not the real calibrated camera).
- The physical arm was detached and reattached rotated ~90° from its
  original mount at some point — fixed in the MOCK's own xacro (see
  below); confirmed via a live pose comparison that the REAL onboard
  robot's own URDF was never actually wrong, so no equivalent fix was
  needed there.

### Two ROS2 workspaces in this devcontainer
- **`/home/ws`** (== host `steve_ros2_ws_student/`) — the TA (Sicong
  Pan) stack, package `neo_mpo_700-2`, no top-level namespace, arm TF
  prefix `ur5` for the MOCK. This is what you're in.
- The onboard robot PC runs a DIFFERENT stack, `steve_hardware_bringup` /
  `steve_essentials` (from TA-handoff repos `steve_essentials-dev` +
  `steve_hardware_bringup-dev`, authors Shrikar Nakhye et al., not present
  in this workspace) — launched via
  `steve_hardware_bringup hardware_bringup.launch.py arm_type:=ur5e`,
  autostarted on boot in a `screen` session named `bringup`
  (`screen -r bringup` to watch, `Ctrl+A D` to detach — never `Ctrl+C`).

### Network — how the dev PC reaches the real robot's live ROS graph
- Onboard PC (`mmo-700-uni-bonn-2204`, user `neobotix`) runs ROS 2 on
  **`ROS_DOMAIN_ID=74`**, `rmw_fastrtps_cpp`, on the **`192.168.60.0/24`**
  subnet (its `enp2s0` NIC, IP `.90`) — NOT `192.168.1.0/24` as an older
  integration doc once assumed for this subsystem.
- SSH: `ssh neobotix@192.168.60.90` or `neobotix@10.7.4.213` (RobotsHRL
  WiFi), password from the TA.
- Dev PC (`knoppers`) side: **`source /home/ws/steve_env.sh`** — adds
  `192.168.60.50/24` to `enp3s0`, exports `ROS_DOMAIN_ID=74
  ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, restarts the
  `ros2` daemon. Idempotent, safe to re-run. Auto-sourced in every new
  shell via `~/.bashrc`, but the manually-added IP does NOT survive a
  container/cable state change — re-run `source /home/ws/steve_env.sh`
  any time `ping 192.168.60.90` stops working.
- Verify: `ros2 topic list` should show ~40 topics;
  `/io_and_status_controller/robot_mode` should read `7` (RUNNING) when
  the arm is actually powered and the safety chain is clear.
- **An older integration note in this project claimed the arm lived at
  `192.168.1.102` and was permanently unreachable there — that's about a
  DIFFERENT internal link (the onboard PC's dedicated point-to-point
  cable to the UR control box's own network port), not the path used
  above. Don't resurrect that framing; the domain-74 path above is how
  this project actually talks to the robot.**

### Live ROS graph reference (arm side)
- Joints: `ur5eshoulder_pan_joint ur5eshoulder_lift_joint ur5eelbow_joint
  ur5ewrist_1_joint ur5ewrist_2_joint ur5ewrist_3_joint`.
- Controller: `/scaled_joint_trajectory_controller` (action
  `.../follow_joint_trajectory`). Starts INACTIVE even with
  `robot_mode=RUNNING` until an **"External Control" program is actively
  running on the pendant** (Play pressed, kept running) — dashboard status
  alone never implies ROS can command the arm; check
  `ros2 control list_controllers` for `active`.
- `/joint_states` has THREE partial publishers merged by
  `steve_robot_state_publisher` (arm `joint_state_broadcaster`
  ~411Hz/`ur5e*` names, base `steve_omnidrive_socketcan_node` ~7Hz, pan-
  tilt) — always filter/index by joint NAME, never assume ordering.
- TF chain: `odom → base_link → cabinet_link → ur5ebase_link →
  ur5ebase_link_inertia → ... → ur5etool0 → dummy_end_effector →
  wrist_d405_mount → wrist_camera_link → wrist_camera_*_optical_frame`
  (the last 3 links are CAD-guessed, unrelated to the real calibrated
  D415 — see hand-eye calibration below).
- Safety status topics (`/emergency_stop_state`,
  `/io_and_status_controller/{robot_mode,safety_mode}`) need **Transient
  Local QoS** to subscribe — they're latched, default Volatile QoS
  silently receives nothing.

### Mock bringup (for RViz-only work when the real robot is unavailable)
Isolated on **`ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1`** so it can never
collide with the real robot's domain-74 topics if that comes back up mid-
session — set this explicitly on every mock-targeted command (direct
`VAR=val cmd` assignment is enough, no need to clear first):
```
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 bash /home/ws/launch_mock.sh
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 bash /home/ws/launch_rviz.sh
```
Mock uses `arm_type:=ur5` → prefix `ur5`, controller
`/joint_trajectory_controller`, no gripper, UR5/CB3-shaped kinematics
(rough sanity check of motion shape, not a precise preview of the ur5e-
shaped real arm). `clean.sh` kills stale ROS processes/daemon — run it
first if `launch_mock.sh` seems wedged; it's already called internally by
`launch_mock.sh`.

**Live robot RViz mirror** (pure viewer, no execution, arm-only filtered
view): `bash /home/ws/launch_live_rviz.sh` — auto-connects via
`steve_env.sh`, uses the saved `steve_live_rviz.rviz` config (now also
shows object pose/obstacle/viewpoint markers by default, not arm-only
anymore — a user edit persisted that).

### ⚠️ CURRENT BLOCKER (since 2026-09-17): back lidar hardware fault
The back SICK S300 unit (`lidar_2`) shows a steady red LED with nothing in
front of it (a second LED on the same unit correctly blinks for a real
person, so the unit isn't fully dead). Confirmed from the live graph:
`/emergency_stop_state.scanner_stop = true`, `safety_mode = 5`
(SAFEGUARD_STOP), `robot_mode = 3` (POWER_OFF), `/lidar_2/scan` produces
ZERO messages (cross-checked against healthy `/lidar_1/scan` and `/tf` in
the same window), and `/rosout` shows `lidar_2.steve_sick_s300_node`
continuously logging `"buffer overflow"` (`neo_sick_s300_node.cpp:107`)
every ~2.6s — meaning the serial link IS receiving signal but never
successfully frames a valid message (signal-integrity fault, not a fully
dead link). **Survived both a cable reseat and a full robot restart** —
rules out "just a stuck process," points at either a damaged
cable/connector or a device-side comm-config fault (needs SICK's own
CDS/SOPAS tool to diagnose further, not available here). TA emailed, none
present to help as of this snapshot.

**No software bypass exists or should be attempted** —
`disable_scanners:=True` only stops the ROS *nodes*, not the hardwired
OSSD safety signal; `robot_mode` stays `POWER_OFF` regardless of any
launch flag. This is a safety-rated circuit; the fix has to be physical.
This blocks REAL arm motion only — it does NOT block CuRobo planning, mock/
RViz work, or camera/perception work if the D415 is still connected
separately. Keep working on everything that doesn't need the arm's motors.

An EARLIER, separate incident (documented for pattern-matching only, TA
has since fixed it): a "Safeguard Stop" from the base's SICK scanners
latching due to a nearby table within their ~15-20cm scan plane — fixed by
moving the table clear / reconfiguring protective fields (needs SICK tool
+ password + TA), same "escalate, don't bypass" playbook.

---

## Real-robot NBV pipeline — CuRobo decision, built pieces, current status

**Decision: CuRobo on the real robot too, not MoveIt2** (a config for that,
`neo_ur_moveit_config`, exists in this workspace as a documented fallback
only — not the plan, since CuRobo's GPU-parallel planning is the actual
point of this Parallel Computing course project).

### Perception plan (object is static for the whole run, confirmed by the user)
1. **Camera calibration — DONE.** D415 eye-in-hand hand-eye calibration,
   Tsai method, cross-validated against 4 other solvers (max 3.3mm/2.3°
   spread) and against held-out poses not used in the solve (3.4mm mean).
   Solved transform (`ur5etool0 → camera`) lives in
   `/home/ws/nbv_scratch/handeye_result.json`
   (`translation_m`, `quaternion_xyzw` — note: this is `T_gripper_camera`,
   i.e. the camera's pose EXPRESSED IN the `tool0` frame). Published live
   via `publish_camera_extrinsic.sh` as a static TF — watch for frame-name
   collisions if extending this: an earlier bug had this extrinsic's child
   frame name collide with the RealSense driver's own internal frame of
   the identical name, making TF lookups nondeterministic; the fix was
   renaming the extrinsic's child frame to something no other publisher
   owns (`*_handeye` suffix), not renamed back since.
2. **SAM3 segmentation — DONE, runs on EVERY captured frame** (not
   one-shot). Via HuggingFace `transformers` (`Sam3Model` + `Sam3Processor`,
   NOT the heavier `facebookresearch/sam3` research repo, NOT plain
   `AutoModel`), its own conda env `sam3` (Python 3.12,
   `torch==2.10.0+cu128` — pin exactly, an unpinned install crashes with
   `Bus error`). Server (`sam3_server.py`, FastAPI, port 8420) + ROS bridge
   (`sam3_ros_bridge.py`, system Python, rclpy). Measured cost: ~1-1.1s/
   call uncontended, ~4.2GB VRAM — doesn't fit alongside FoundationPose on
   this 8GB GPU at the same time. Query-wording lesson: prefer generic
   shape/material terms ("carton box") over product-category terms ("cereal
   box") since SAM3 is an appearance matcher, not a shape detector — fails
   when the visible face doesn't look branded.
3. **Object 6D pose — DONE, runs ONCE per run** (object never moves).
   **FoundationPose was tried first, fully installed/verified, but set
   aside for THIS object** (not abandoned as infrastructure — revisit for
   a future textured object): the real inspection object is a plain,
   UNTEXTURED consumer snack box with no CAD model, and FoundationPose's
   learned refiner needs texture to disambiguate similar rotations —
   locked onto a stable but wrong orientation. **Replaced with a classical
   SAM3+PCA+ICP tracker** (`box_icp_ros_bridge.py`'s `BoxIcpTracker` class,
   pure geometry, no rclpy dependency, reusable standalone), matching the
   user's actual requirement ("doesn't matter which way the box faces, only
   that the pose stays consistent"). Fixed local-axis convention:
   `R = column_stack([thin_axis, mid_axis, wide_axis])` — i.e. box-local
   x=thin, y=mid, z=wide dimension — this convention is load-bearing for
   any other code that reasons about the object's local frame (e.g. the
   coverage-points generator in `full_pipeline/step3_...`, which
   deliberately matches it). One-shot wrapper: `estimate_object_pose.py`
   retries until several consecutive estimates agree, then writes
   `/home/ws/nbv_scratch/cereal_box/object_pose.json`
   (`t_arm_base`/`q_arm_base_xyzw`, ALREADY in the arm's `ur5ebase_link`
   frame, `dims_m` sorted `[thin, mid, wide]`).

### CuRobo real-robot scripts (all `/home/ws`, git-ignored, `rob_env` conda
### env — Python 3.12, CuRobo `v0.7.8`, `torch==2.4.1+cu121`)
- **Hard constraint, applies to every script below**: `rclpy` is ABI-locked
  to the SYSTEM Python 3.10 — no PyPI wheel exists at all. It can NEVER
  share a process with CuRobo (`rob_env`, Python 3.12). Every CuRobo script
  reads/writes plain JSON files; a separate system-Python/rclpy script does
  the actual ROS talking. This split is permanent, not a temporary
  workaround.
- `ros_joint_state_dump.py` (rclpy) — one-shot: dumps the arm's CURRENT
  joint state to a JSON CuRobo can read as a start state. `--prefix ur5e`
  (live) or `ur5` (mock).
- `curobo_plan_real_spike.py` (rob_env) — joint-space (`plan_single_js`)
  planning, `--offset` or `--goal <file>`, auto-loads table + object
  `Cuboid` obstacles if their pose files exist (`--no-floor`/`--no-object`
  to disable). `CollisionCheckerType.PRIMITIVE` always.
- `ros_send_trajectory.py` (rclpy) — sends a planned trajectory as ONE
  multi-waypoint `FollowJointTrajectory` goal, waits for the REAL result
  (not just acceptance — this fix is what prevents a "Pipeline Producer
  overflowed" `controller_manager` crash seen once when rapid-tapping jog
  keys overlapped goals). `--target mock|live`, `--slow-factor` (default
  8x, pure time-reparametrization, only ever reduces peak vel/accel — safe
  to lower toward 1.0 but not below). `--target live` requires
  `robot_mode=RUNNING` + `safety_mode=NORMAL` + typing `go`.
- `run_benchmark_pose.sh` — `--pose <name> --target mock|live`, ALWAYS
  reads the REAL robot's live pose as the plan's start (even for
  `--target mock`, which does an extra real→mock sync first — mock's
  trajectory controller otherwise rejects a goal whose first waypoint
  doesn't match its own last-known state). Saved poses in
  `nbv_scratch/benchmark_poses.json`: `hardest` (near a joint-limit edge),
  `second_hardest` (clean diagonal reach), `zero_easiest` (NOT literal
  all-zeros — `shoulder_lift≈-pi/2, wrist_1≈-pi/2`, rest ≈0; the user calls
  this "the zero position," easy to confuse with `move_arm.py`'s literal
  `zero` which IS all-zeros).
- `floor_geometry.py` / `object_geometry.py` — shared, dependency-free
  (stdlib/JSON, no numpy) obstacle-geometry modules, imported by BOTH the
  `rob_env` planning scripts and the system-Python marker publishers below,
  so planned-against and visualized geometry can never drift apart.
  Current table numbers: `0.5×0.5m` footprint, `1.0m` thick, flush with
  the arm's own `base_link` origin (`offset=0`, margin=0 — CuRobo's
  collision checker tolerates far less clearance than the naive
  sphere-radius math predicts; tested empirically down to exactly 0, don't
  reintroduce a defensive margin without re-testing). Object obstacle:
  padded +2cm on every side of the ICP-estimated dims (`OBJECT_MARGIN_M`).
- `publish_floor_marker.py` / `publish_object_marker.py` /
  `publish_viewpoint_markers.py` (rclpy) — RViz visualization of what
  CuRobo is actually planning around, `--prefix ur5`/`ur5e`. Object marker
  publishes both the raw estimate (yellow) and the padded collision cuboid
  (red). Viewpoint markers: green=reachable+collision-free,
  yellow=reachable but blocked by table/object, red=unreachable.
- `save_pose.py` (rclpy) — appends the robot's current pose to
  `benchmark_poses.json`.
- `mirror_real_to_mock.sh` — one command: real pose → mock pose → CuRobo
  plan between them → send to mock, so RViz mirrors the real robot's
  current pose. Reference pattern for mixing `conda activate rob_env` and
  plain system-Python `rclpy` calls within one bash script — the same
  pattern `full_pipeline/run_nbv_loop.sh` (below) follows.

### `/home/ws/full_pipeline/` — the final real-robot NBV orchestration, DONE and mock-validated
Three standalone steps, per the user's own explicit framing:
1. **Object pose** — `/home/ws/estimate_object_pose.py` (above).
2. **Reachability map** — `/home/ws/sample_object_viewpoints.py` (rob_env):
   samples a hemisphere shell of candidate CAMERA poses around the known
   object position (full 2π azimuth, IK naturally rejects the unreachable
   far side), composes each through the INVERSE of the hand-eye transform
   to get the `tool0` target CuRobo's IK solver actually needs (CuRobo's
   `ee_link` is `tool0` — there is no camera in its kinematic chain, and
   per project decision the URDF is NOT edited to add one), batch-solves
   IK twice (kinematic-only, and collision-aware with the table+object
   obstacles loaded) via `IKSolverConfig`, writes
   `nbv_scratch/cereal_box/viewpoint_cache.json`. **Important, real
   gotcha**: its look-at orientation convention is DELIBERATELY different
   from the sim's — the sim uses a Y-up/X-left convention (whatever
   PyBullet's virtual camera wanted); the real D415's `color_optical_frame`
   is the standard ROS/OpenCV optical convention (+Z forward, +X right, +Y
   down), which is what the hand-eye calibration was actually solved
   against. Copying the sim's formula unchanged would silently roll every
   candidate 180° about its own viewing axis.
3. **Motion planning + NBV** — `full_pipeline/step3_nbv_motion_planning.py`
   + `commit_capture.py` + `run_nbv_loop.sh`. Greedy NBV: score every
   reachable/unvisited candidate by how many currently-unseen points on a
   PROCEDURALLY-built box mesh (from `object_pose.json`'s `dims_m`, no
   trimesh dependency needed) it would newly reveal (GPU-batched
   Moller-Trumbore ray/triangle visibility test, front-facing +
   not-self-occluded), pick the best, Cartesian-plan (`MotionGen.
   plan_single`, not joint-space) a collision-aware trajectory there from
   the arm's ACTUAL current pose. The object's own resting/bottom face
   (whichever local face ends up most -Z-aligned after its solved
   rotation) is excluded from the coverage target entirely — it's
   permanently unseeable, chasing 100% of it is meaningless.
   - **No live camera → dummy capture, an explicit, deliberate stand-in**:
     since the ray-visibility scoring function already predicts exactly
     what would be seen from any pose, `commit_capture.py` just trusts that
     prediction instead of a real depth backprojection. Correct in the
     noiseless-sensor limit; validates everything BUT real sensor/
     segmentation accuracy. **Real depth integration is the agreed next
     step, not started**: replace `commit_capture.py` with a real D415
     capture → SAM3 mask → backproject → nearest-neighbor match against
     the known mesh points. Doesn't need the arm's motors, only the camera
     + SAM3 — can start independent of the lidar blocker above if the
     D415 is still connected.
   - **TRUE closed-loop, not open-loop-assumed**: each iteration of
     `run_nbv_loop.sh` freshly reads the arm's REAL current pose
     (`ros_joint_state_dump.py`), plans ONE step from it (`step3`,
     stateful — progress persisted to `nbv_scratch/cereal_box/
     nbv_state.json` between separate process invocations, since
     CuRobo/rclpy can never share a process), sends it and WAITS for real
     confirmed completion, and only THEN commits the capture
     (`commit_capture.py`) — never assumes a planned move actually
     succeeded. If a send ever fails, the loop stops without crediting
     that view, rather than silently continuing from a wrong assumed pose.
   - Run: `bash /home/ws/full_pipeline/run_nbv_loop.sh --target mock` (or
     `--target live` once the robot's back; `--resume` to continue an
     existing tour instead of resetting). Verified live end-to-end on the
     mock 2026-09-18: 2-3 iterations, each with a genuinely different
     freshly-read start pose, reaching 100% (non-excluded) coverage
     cleanly.

---

## Diagnostic / debugging workflow

Claude has direct shell access in this devcontainer (bash, `ip`, `sudo`,
`ros2`, `git`, `conda`) — an old note claiming otherwise is wrong, ignore
it. For the ONBOARD PC specifically (no direct shell access there), the
workflow is: Claude writes a diagnostic script, the user runs it via SSH
and pastes output, or (preferred, already proven) query the onboard PC's
live ROS graph directly from the dev PC over the domain-74 connection
above — this covers almost everything needed without ever SSHing in.

- **HARD `timeout` on every `ros2` call** — `ros2 control ...` and similar
  hang forever when `controller_manager` isn't answering (often the exact
  failure being diagnosed).
- **`ros2 <cmd> --no-daemon`** — the `ros2cli` daemon corrupts/goes stale
  easily in this container; `--no-daemon` sidesteps it for one-shot calls.
  `tf2_echo`/`view_frames` make their own node and stay reliable even when
  the daemon is dead.
- **Cold multicast discovery is genuinely flaky even when healthy** — the
  exact same `ros2 topic list`/`node list` call can return the full graph,
  then nothing, then the full graph again across consecutive tries within
  seconds. Retry 2-3 times before concluding anything is actually down —
  don't trust a single empty result.
- **A live, long-running Bash-tool `ros2` CLI session can itself start
  reliably discovering endpoints (`topic list`/`info`) while reliably
  failing to receive actual DATA (`topic hz`/`echo`)** after many prior
  short-lived `ros2` invocations in the same session — a real, still
  unexplained tooling quirk, not evidence the publisher is broken. Cross-
  check any "received nothing" result against (a) the publishing script's
  own logs/counters, (b) the user's own long-running GUI viewer if one's
  open, and (c) whether a COMPLETELY UNRELATED, definitely-healthy topic
  also fails the same check in the same moment — only trust the failure if
  that control check fails too.
- **`pkill` with no match kills the entire Bash tool call**, even with
  `|| true` after it — not normal bash semantics, appears specific to this
  sandbox's interception of `pkill`. Use `kill -9 <pid>` from a prior `ps`
  lookup instead when Claude itself runs the command and can't guarantee a
  match exists.
- To iterate on a launch/xacro fix WITHOUT a full `colcon build`: patch
  BOTH the `src/` file and its `install/.../share/...` copy (the install
  tree here is NOT symlink-installed — editing only `src/` silently does
  nothing until a real `colcon build --packages-select <pkg>`).
- Finding a topic's real publisher when several share a name: `ros2 topic
  info /<topic> --verbose` lists each publisher/subscriber with node name +
  GID + QoS; match an empty-node-name GID against `ros2 node list`
  candidates if needed.
- **RViz `Marker` vs `MarkerArray` on the same topic name is a real crash
  bug, not a clean rejection** — if two displays in the SAME RViz process
  ever subscribe to one topic name expecting two different message types
  (e.g. leftover `Marker` display + newly-added `MarkerArray` display),
  `rclcpp` throws `invalid allocator` and RViz aborts (core dump). Always
  fully remove a wrong-type display before adding the correct one, don't
  leave both present even briefly.
- **Sending motion to the mock is zero physical risk** — fine for Claude
  to do directly (via `ros_send_trajectory.py --target mock` or
  `move_arm.py`) to verify its own work, unlike anything targeting the
  live arm.

---

## Diagnostic / control scripts (all `/home/ws`, git-ignored)

**Arm control (mock):** `move_arm.py` (single-point move, named poses
`zero`/`up`/`ready` or 6 explicit angles), `jog_joint.py` (interactive
per-joint jog).
**Arm control (live):** `jog_arm_live.py` (same UX as `jog_joint.py` but
`ur5e`-safe — safety-gated, waits for goal RESULT not just acceptance),
`watch_arm_pose.py` (read-only pose-change watchdog, sends nothing).
**CuRobo pipeline:** see the dedicated section above
(`ros_joint_state_dump.py`, `curobo_plan_real_spike.py`,
`ros_send_trajectory.py`, `run_benchmark_pose.sh`, `mirror_real_to_mock.sh`,
`save_pose.py`, `floor_geometry.py`/`object_geometry.py` +
their marker publishers).
**Perception:** `handeye_capture.py`/`handeye_solve.py`/
`handeye_holdout_check.py`/`publish_camera_extrinsic.sh` (calibration),
`sam3_segmenter.py`/`sam3_server.py`/`sam3_ros_bridge.py` (segmentation),
`box_icp_ros_bridge.py` (the `BoxIcpTracker` class + a live-tracking
bridge — mostly superseded by the one-shot `estimate_object_pose.py` for
this project's actual "object never moves" use case, but the class itself
is still what's reused), `make_box_mesh.py` (generates the untextured
stand-in mesh), `foundationpose_*.py` (installed, working, set aside for
this object — see above).
**Full pipeline:** `full_pipeline/plan_to_viewpoint.py` (single-candidate
Cartesian plan, superseded by `step3_...` for the actual NBV loop but
useful for one-off manual tests), `full_pipeline/step3_nbv_motion_planning.py`,
`full_pipeline/commit_capture.py`, `full_pipeline/run_nbv_loop.sh`.
**Camera:** `fix_d415_camera.sh` (re-creates `/dev` nodes after a USB
replug — this container's `/dev` doesn't live-sync hotplug events),
`live_frame_saver.py` (no-GUI fallback viewer). Use `rqt_image_view` to
actually watch a live topic — plain `image_view`'s window never maps in
this container's X11 setup (a real, confirmed bug, not user error).
**Networking/diagnostics:** `steve_env.sh` (the domain-74 connect script,
above), `clean.sh` (kill stale ROS processes + daemon), various one-off
`*_probe.sh`/`diag.sh` scripts from earlier network debugging — mostly
historical, `steve_env.sh` + `clean.sh` cover current needs.

All of `/home/ws/nbv_scratch/` is scratch output (poses, trajectories,
calibration results, captured frames, logs) — git-ignored, safe to inspect
freely, safe to delete/regenerate individual files if something looks
stale (many scripts print exactly which file they wrote).
