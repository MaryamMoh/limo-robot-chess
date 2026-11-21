#!/usr/bin/env python3
"""Spawns the chess pieces in Gazebo and acts as the robots' "gripper".

The LIMO PRO has no manipulator, so in simulation a carried piece is placed
on the robot's carrying plate and kept there while the robot drives (the
piece pose is updated from the robot's Gazebo pose at `rate` Hz).

Services (robot_chess_msgs):
    ~attach   AttachPiece   piece -> top of robot
    ~release  ReleasePiece  carried piece -> floor at (x, y)
    ~replace  ReplacePiece  swap a piece for another (promotion)
    ~reset    std_srvs/Trigger  put all 32 pieces back on their start squares

Parameters:
    ~board          board geometry (see robot_chess_game/config/board_sim.yaml)
    ~carry_height   height of the carried piece's bottom above the floor [m]
    ~rate           update rate for carried pieces [Hz]
"""
import math

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import (DeleteModel, GetWorldProperties, SetModelState,
                             SpawnModel)
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Trigger, TriggerResponse

from robot_chess.board_geometry import BoardGeometry
from robot_chess.piece_models import piece_sdf
from robot_chess.pieces import initial_square_models
from robot_chess_msgs.srv import (AttachPiece, AttachPieceResponse,
                                  ReleasePiece, ReleasePieceResponse,
                                  ReplacePiece, ReplacePieceResponse)


def yaw_quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimPieceManager(object):

    def __init__(self):
        self.geometry = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.carry_height = rospy.get_param("~carry_height", 0.26)
        self.rate = rospy.get_param("~rate", 30.0)

        self.model_poses = {}       # gazebo model name -> Pose
        self.carried = {}           # piece model -> robot model

        for srv in ("/gazebo/spawn_sdf_model", "/gazebo/delete_model",
                    "/gazebo/set_model_state", "/gazebo/get_world_properties"):
            rospy.loginfo("waiting for %s", srv)
            rospy.wait_for_service(srv)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.world_props = rospy.ServiceProxy("/gazebo/get_world_properties",
                                              GetWorldProperties)
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState,
                                         queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_model_states,
                         queue_size=1)

        self.reset_board()

        rospy.Service("~attach", AttachPiece, self.on_attach)
        rospy.Service("~release", ReleasePiece, self.on_release)
        rospy.Service("~replace", ReplacePiece, self.on_replace)
        rospy.Service("~reset", Trigger, self.on_reset)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.update_carried)
        rospy.loginfo("sim piece manager ready")

    # ----------------------------------------------------------- spawning
    def _spawn_piece(self, model, color, ptype, x, y):
        pose = Pose()
        pose.position.x, pose.position.y = x, y
        pose.orientation.w = 1.0
        resp = self.spawn(model_name=model, model_xml=piece_sdf(model, color, ptype),
                          robot_namespace="", initial_pose=pose, reference_frame="world")
        if not resp.success:
            rospy.logerr("spawning %s failed: %s", model, resp.status_message)
        return resp.success

    def _delete_if_present(self, model):
        if model in self.world_props().model_names:
            self.delete(model_name=model)

    def reset_board(self):
        self.carried.clear()
        existing = set(self.world_props().model_names)
        for name in existing:
            if name.startswith(("white_", "black_")):
                self.delete(model_name=name)
        for square, model in sorted(initial_square_models().items()):
            color, ptype, _ = model.split("_", 2)
            x, y = self.geometry.square_center(square)
            self._spawn_piece(model, color, ptype, x, y)
        rospy.loginfo("spawned 32 chess pieces")

    # ------------------------------------------------------------ carrying
    def on_model_states(self, msg):
        self.model_poses = dict(zip(msg.name, msg.pose))

    def update_carried(self, _event):
        for piece, robot in list(self.carried.items()):
            rp = self.model_poses.get(robot)
            if rp is None:
                continue
            yaw = quaternion_yaw(rp.orientation)
            st = ModelState()
            st.model_name = piece
            st.reference_frame = "world"
            # slightly behind the robot centre so the camera view stays free
            st.pose.position.x = rp.position.x - 0.03 * math.cos(yaw)
            st.pose.position.y = rp.position.y - 0.03 * math.sin(yaw)
            st.pose.position.z = self.carry_height
            st.pose.orientation = yaw_quaternion(yaw)
            self.state_pub.publish(st)

    def _place(self, model, x, y):
        st = ModelState()
        st.model_name = model
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y = x, y
        st.pose.orientation.w = 1.0
        return self.set_state(model_state=st)

    # ------------------------------------------------------------ services
    def on_attach(self, req):
        if req.model_name not in self.model_poses:
            return AttachPieceResponse(False, "unknown piece %s" % req.model_name)
        if req.robot_name not in self.model_poses:
            return AttachPieceResponse(False, "unknown robot %s" % req.robot_name)
        self.carried[req.model_name] = req.robot_name
        rospy.loginfo("%s picked up %s", req.robot_name, req.model_name)
        return AttachPieceResponse(True, "attached")

    def on_release(self, req):
        if self.carried.pop(req.model_name, None) is None:
            return ReleasePieceResponse(False, "%s is not carried" % req.model_name)
        # make sure no queued update moves the piece back onto the robot
        rospy.sleep(2.0 / self.rate)
        resp = self._place(req.model_name, req.x, req.y)
        rospy.loginfo("%s put %s down at (%.2f, %.2f)", req.robot_name,
                      req.model_name, req.x, req.y)
        return ReleasePieceResponse(resp.success, resp.status_message)

    def on_replace(self, req):
        self.carried.pop(req.old_model_name, None)
        self._delete_if_present(req.old_model_name)
        ok = self._spawn_piece(req.new_model_name, req.color, req.piece_type, req.x, req.y)
        return ReplacePieceResponse(ok, "replaced" if ok else "spawn failed")

    def on_reset(self, _req):
        self.reset_board()
        return TriggerResponse(True, "board reset")


if __name__ == "__main__":
    rospy.init_node("chess_carrier")
    SimPieceManager()
    rospy.spin()
