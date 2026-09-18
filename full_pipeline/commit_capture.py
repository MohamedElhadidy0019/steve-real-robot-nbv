#!/usr/bin/env python3
"""
Run AFTER run_nbv_loop.sh has confirmed the just-sent move actually
succeeded (ros_send_trajectory.py exited 0, meaning the real
FollowJointTrajectory goal was accepted AND finished, not just accepted --
see that script's docstring). Commits step3_nbv_motion_planning.py's
"pending" prediction (which candidate, which mesh points it predicted
visible) into the persisted seen/visited state.

This is the ONE place a real camera capture + segmentation + backprojection
would go instead, once one is available -- right now it just trusts the
ray-visibility prediction already computed at planning time, which is
correct in the noiseless-sensor limit this whole exercise assumes.

Deliberately separate from planning: if the send FAILS (safety stop,
controller fault -- both have actually happened in this project), this
script must NOT be called, so the candidate stays unvisited and its points
stay unseen for a future attempt, rather than crediting a view the arm
never actually reached.

No CuRobo, no rclpy -- plain stdlib, can run anywhere.

Run:
    python3 commit_capture.py --state /home/ws/nbv_scratch/cereal_box/nbv_state.json
"""
import argparse
import json


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default="/home/ws/nbv_scratch/cereal_box/nbv_state.json")
    args = ap.parse_args()

    with open(args.state) as f:
        state = json.load(f)

    pending = state.get("pending")
    if pending is None:
        print("! no pending capture in state -- nothing to commit (already committed, "
              "or step3 was never run first)")
        raise SystemExit(1)

    seen = state["seen"]
    for i, was_visible in enumerate(pending["visible_mask"]):
        if was_visible:
            seen[i] = True
    state["seen"] = seen
    state["visited"].append(pending["candidate"])
    state["pending"] = None

    with open(args.state, "w") as f:
        json.dump(state, f)

    n_seen = sum(seen)
    print(f"committed candidate {pending['candidate']} ({pending['new_points']} new points) "
          f"-- total coverage now {n_seen}/{len(seen)} ({n_seen / len(seen):.1%})")


if __name__ == "__main__":
    main()
