"""
Shared obstacle geometry -- single source of truth for both
curobo_plan_real_spike.py (rob_env, Python 3.12, uses this for planning) and
publish_floor_marker.py (system Python 3.10, rclpy, uses this to visualize
the same box in RViz). Deliberately dependency-free (stdlib only) so both
Python versions can import it unchanged.

This is a deliberately SIMPLE proxy: per the user (2026-09-11), model it as if
the arm were bolted DIRECTLY to a 4x4m table -- no cabinet, no mobile-base
gap in between (supersedes an earlier version of this file that modeled the
table as sitting at the Neobotix base's own top, 0.766m below the arm). So
conceptually/visually the table top is FLUSH with the arm's own base_link
origin (offset 0).

But the COLLISION proxy can't actually sit exactly at offset 0: ur5e.yml's
own shoulder_link collision sphere is centered at the arm's base_link origin
with radius 0.1m, so a table flush with that origin would always overlap it
-- every start state would register INVALID_START_STATE_WORLD_COLLISION and
CuRobo would refuse to plan at all. This exact failure mode already happened
once in the sim pipeline (nbv_core/motion_planning.py's table) and was fixed
the same way: keep the true/visual mounting flush, lower only the collision
proxy a few cm (currently ~3cm -- tested empirically, not derived from the
0.1m sphere radius; CuRobo's checker tolerates closer than the naive math
predicts, see [[steve-table-obstacle-model]] memory for the full history).
"""

TRUE_MOUNT_OFFSET = 0.0  # arm bolted directly to the table -- flush, no gap
COLLISION_MARGIN = 0.0   # 2026-09-14: user asked to lift the box's CENTER up
                          # 3cm again (shape/thickness unchanged) -- since
                          # thickness stayed fixed, that's mathematically the
                          # same as dropping the margin straight to 0, i.e.
                          # top surface now EXACTLY flush with the arm's base
                          # origin. Tested empirically before trusting this
                          # (see steve-table-obstacle-model memory) -- this
                          # is a step further than anything tested so far
                          # (previous tests only went down to 0.02m margin,
                          # never all the way to 0). Re-verify after any
                          # further change here.
DEFAULT_BASE_TABLE_OFFSET = TRUE_MOUNT_OFFSET + COLLISION_MARGIN  # 0.03m
DEFAULT_BASE_TABLE_SIZE = 0.5  # 2026-09-14: user shrank this from 4x4m to
                                # 0.5x0.5m, arm-centered -- a much more
                                # plausible footprint for the Neobotix base
                                # itself than the original 4x4m "big work
                                # table" framing
BASE_TABLE_THICKNESS = 1.0  # 2026-09-14: raised from 0.2m to 1.0m -- user
                             # wants the box tall enough to visually reach
                             # down near the actual ground, not just a thin
                             # slab right under the arm. Only ~2-3cm of
                             # margin beyond bare contact with shoulder_link's
                             # own sphere (see the module docstring) --
                             # RE-VERIFY planning still succeeds after any
                             # change here, a much taller box can intersect
                             # other links during larger motions that the
                             # thin 0.2m slab never touched.

# Old names kept as aliases -- curobo_plan_real_spike.py / publish_floor_marker.py
# still import these; not worth a mechanical rename pass across all three files.
DEFAULT_FLOOR_OFFSET = DEFAULT_BASE_TABLE_OFFSET
DEFAULT_FLOOR_SIZE = DEFAULT_BASE_TABLE_SIZE
FLOOR_THICKNESS = BASE_TABLE_THICKNESS


def floor_center_z(offset=DEFAULT_BASE_TABLE_OFFSET, thickness=BASE_TABLE_THICKNESS):
    """Z of the box's CENTER (what CuRobo's Cuboid pose / a Marker pose both want),
    in the arm base_link frame, given offset = meters from the arm's base_link
    DOWN to the Neobotix base's own top surface."""
    top_z = -offset
    return top_z - thickness / 2
