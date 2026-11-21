#!/usr/bin/env python3
"""Check a board layout before trying it on the robot.

    check_reach.py [config.yaml] [--tcp-offset 0.06]

For every square and graveyard slot, at grasp and at travel height, and for
the camera observe pose, solve the arm IK with the tool pointing down and
print a map of what the arm can reach.  Runs without ROS.
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src"))

import chess  # noqa: E402

from robot_chess.board_geometry import BoardGeometry  # noqa: E402
from robot_chess.kinematics import MyCobotKinematics  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("config", nargs="?", default=os.path.join(
        os.path.dirname(HERE), "config", "player_sim.yaml"))
    ap.add_argument("--tcp-offset", type=float, default=None)
    args = ap.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    g = BoardGeometry.from_dict(cfg["board"])
    arm = cfg.get("arm", {})
    k = MyCobotKinematics(args.tcp_offset if args.tcp_offset is not None
                          else arm.get("tcp_offset", 0.06))

    def ok(label):
        return k.ik_down(g.grasp_point(label)) is not None and \
            k.ik_down(g.travel_point(label)) is not None

    bad = []
    print("reachable squares (o) / unreachable (X), rank 8 at the top:")
    for rank in range(7, -1, -1):
        row = []
        for f in range(8):
            name = chess.square_name(chess.square(f, rank))
            good = ok(name)
            row.append("o" if good else "X")
            if not good:
                bad.append(name)
        print("  %d %s" % (rank + 1, " ".join(row)))
    print("    a b c d e f g h")
    graves = ["graveyard-%d" % i for i in range(g.graveyard_slots)]
    bad_graves = [lab for lab in graves if not ok(lab)]
    observe = k.ik_down(g.observe_point(), yaw=arm.get("observe_yaw", 0.0)) is not None
    print("graveyard slots unreachable: %s" % (bad_graves or "none"))
    print("observe pose reachable: %s" % observe)
    if bad or bad_graves or not observe:
        print("=> layout NOT usable with a fixed base")
        return 1
    print("=> layout OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
