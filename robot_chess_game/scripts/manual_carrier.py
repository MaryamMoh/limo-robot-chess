#!/usr/bin/env python3
"""Carrier for the real LIMO PRO (no manipulator on board).

Provides the same services as the simulated carrier (sim_piece_manager.py)
so piece_mover works unchanged.  When the robot has arrived at a piece, an
assistant puts the piece on the robot's top plate; at the destination the
assistant puts it on the square.  The instruction is printed, published on
/chess/operator_instructions and the robot waits until

    rosservice call /chess_carrier/confirm      (or the ~auto_confirm timeout)

Services:  ~attach  ~release  ~replace  ~confirm (std_srvs/Trigger)
"""
import threading

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from robot_chess_msgs.srv import (AttachPiece, AttachPieceResponse, ReleasePiece,
                                  ReleasePieceResponse, ReplacePiece,
                                  ReplacePieceResponse)


def pretty(model):
    parts = model.split("_")
    return "%s %s" % (parts[0].upper(), parts[1].upper()) if len(parts) > 1 else model


class ManualCarrier(object):

    def __init__(self):
        self.auto_confirm = rospy.get_param("~auto_confirm", 0.0)   # 0 = wait forever
        self.event = threading.Event()
        self.pub = rospy.Publisher("/chess/operator_instructions", String, queue_size=5,
                                   latch=True)
        rospy.Service("~attach", AttachPiece, self.on_attach)
        rospy.Service("~release", ReleasePiece, self.on_release)
        rospy.Service("~replace", ReplacePiece, self.on_replace)
        rospy.Service("~confirm", Trigger, self.on_confirm)
        rospy.loginfo("manual carrier ready - confirm with: rosservice call %s/confirm",
                      rospy.get_name())

    def ask(self, text):
        self.event.clear()
        banner = "=" * 60
        rospy.logwarn("\n%s\nOPERATOR: %s\n%s", banner, text, banner)
        self.pub.publish(String(text))
        timeout = self.auto_confirm if self.auto_confirm > 0 else None
        done = self.event.wait(timeout)
        self.pub.publish(String(""))
        return done or self.auto_confirm > 0

    def on_confirm(self, _req):
        self.event.set()
        return TriggerResponse(True, "confirmed")

    def on_attach(self, req):
        ok = self.ask("put the %s in front of the robot on its top plate" % pretty(req.model_name))
        return AttachPieceResponse(ok, "loaded" if ok else "not confirmed")

    def on_release(self, req):
        ok = self.ask("take the %s from the robot and put it at (%.2f, %.2f) - the square "
                      "in front of the robot" % (pretty(req.model_name), req.x, req.y))
        return ReleasePieceResponse(ok, "placed" if ok else "not confirmed")

    def on_replace(self, req):
        ok = self.ask("promotion: replace the %s by a %s %s" % (
            pretty(req.old_model_name), req.color.upper(), req.piece_type.upper()))
        return ReplacePieceResponse(ok, "replaced" if ok else "not confirmed")


if __name__ == "__main__":
    rospy.init_node("chess_carrier")
    ManualCarrier()
    rospy.spin()
