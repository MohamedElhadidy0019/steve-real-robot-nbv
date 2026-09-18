# Steve — Real-Robot NBV Inspection Pipeline

Next-Best-View (NBV) inspection pipeline for a UR5e arm on a Neobotix
MPO-700 mobile base ("Steve", University of Bonn Humanoid Robots Lab).
Real-robot port of a working PyBullet sim pipeline — CuRobo GPU motion
planning, SAM3 segmentation, D415 eye-in-hand hand-eye calibration, a
classical PCA+ICP object pose tracker, and a closed-loop NBV orchestration
loop.

## Start here: `CLAUDE.md`

**[`CLAUDE.md`](./CLAUDE.md) is this project's memory file** — the
persistent context covering hardware, network setup, every script in this
repo, exact conda env versions/gotchas, and the current status (including
whatever hardware blocker is active). Read it before anything beyond "how
do I run the pipeline" below — it's written to be self-contained enough
that anyone (or any agent) can pick the project up from it alone.

## Prerequisites

- ROS 2 Humble, this workspace sourced (`install/setup.bash` after a
  `colcon build`, or use the pre-built `install/` if present).
- Two conda environments (exact install steps + version pins in
  `CLAUDE.md`):
  - **`rob_env`** (Python 3.12) — CuRobo `v0.7.8`, `torch==2.4.1+cu121`.
    Used for all motion planning (steps 2 and 3 below).
  - **`sam3`** (Python 3.12) — segmentation, only needed for step 1
    (estimating a fresh object pose from a live camera).
- **`rclpy` only exists for system Python 3.10** — it's ABI-locked, can't
  be installed into either conda env above. This is a hard, permanent
  constraint in this project, not a bug: every script below already knows
  which environment it needs; just run each one where it says to.

## Quick start — run the full NBV tour on the mock arm (no real robot needed)

This reproduces exactly what was last run and validated end-to-end. It
uses the object pose and reachability cache already checked into
`nbv_scratch/cereal_box/`, so steps 1 and 2 further down don't need to be
re-run first.

**1. Launch the mock arm + RViz** (two terminals; isolated on
`ROS_DOMAIN_ID=0` so it can never collide with a real robot's topics on
the same network):
```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 bash launch_mock.sh
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 bash launch_rviz.sh
```

**2. Run the closed-loop NBV tour:**
```bash
bash full_pipeline/run_nbv_loop.sh --target mock
```
Watch RViz — the arm steps through a few viewpoints around the object,
each one picked live by the NBV planner, until coverage is complete.
`run_nbv_loop.sh` handles everything on its own: reading the arm's real
current pose, planning the next best view, sending the move, and waiting
for confirmed completion before picking the next one — it never assumes a
planned move actually succeeded.

Swap `--target mock` for `--target live` to run this against the real
robot instead (adds a per-move safety confirmation prompt) — no other
changes needed, same script.

## The full pipeline, from scratch (3 standalone steps)

The user's own framing for this project: three independent, standalone
scripts. Useful if you want to regenerate everything — e.g. for a
different object, or after re-running hand-eye calibration.

**Step 1 — estimate the object's pose** (system Python, needs the D415
camera physically connected, the hand-eye calibration already published,
and `sam3_server.py` running — see `CLAUDE.md` for the full startup
sequence):
```bash
source steve_env.sh
python3 estimate_object_pose.py
```
Retries until several consecutive pose estimates agree, then writes
`nbv_scratch/cereal_box/object_pose.json`.

**Step 2 — build the reachability map** (`rob_env`, needs step 1's output;
pure computation, no live camera needed):
```bash
conda activate rob_env
python3 sample_object_viewpoints.py
```
Samples ~360 candidate camera viewpoints in a shell around the object,
batch-checks each with CuRobo — both kinematically and collision-aware
against the table + object — and writes
`nbv_scratch/cereal_box/viewpoint_cache.json`. Visualize the result:
```bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 python3 publish_viewpoint_markers.py --prefix ur5
```
(add a **MarkerArray** display on `/viewpoint_markers` in RViz — green =
reachable + collision-free, yellow = reachable but blocked, red =
unreachable)

**Step 3 — motion planning + NBV** (`full_pipeline/` — this is the same
tour runner used in Quick Start above):
```bash
bash full_pipeline/run_nbv_loop.sh --target mock   # or --target live
```
Greedily scores every reachable, not-yet-visited candidate by how many
currently-unseen points on the known object mesh it would newly reveal,
plans a real CuRobo Cartesian trajectory to the best one, sends it, waits
for real confirmed arrival, marks those points seen, repeats until
coverage is done.

**Known limitation, by design, not yet closed**: there's no live camera
integration yet. "What was actually seen" is currently a geometric
stand-in — the exact same ray-visibility model used to score candidates
in the first place, trusted as ground truth — not a real depth capture.
This validates every other part of the loop (reachability, greedy
selection, Cartesian planning, obstacle avoidance, the real-pose closed
loop) but not real sensor/segmentation accuracy. See `CLAUDE.md` for what
replacing it with a real D415 + SAM3 capture involves.

## Current status

See `CLAUDE.md` for the live/current blocker status. As of the last
snapshot, the real arm was blocked by an unresolved hardware fault
(escalated to the TAs) — everything above is fully built and verified on
the mock in the meantime, ready to point at the real robot the moment
that's fixed.
