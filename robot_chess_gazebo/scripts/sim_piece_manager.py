#!/usr/bin/env python3
"""Chess pieces in Gazebo: spawning, "virtual grasp" and ground truth.

Grasping small static objects with simulated fingers is unreliable in ODE,
so a closed gripper holds a piece kinematically: on ~gripper/attach the
piece nearest to the tool centre point (within 1.5 cm) is attached and then
follows the TCP until ~gripper/release puts it down, upright, on the
surface below.  The arm still has to bring the gripper exactly to the piece:
nothing is attached if it is not there.

Services
    ~gripper/attach    std_srvs/Trigger    grasp the piece at the TCP
    ~gripper/release   std_srvs/Trigger    put the held piece down
    ~replace           robot_chess_msgs/ReplacePiece  promotion (piece at the TCP)
    ~reset             std_srvs/Trigger    all 32 pieces back to the start position
Topics
    /chess_sim/occupancy  std_msgs/String  ground-truth occupancy (a1..h8)

Parameters: ~board (geometry, board pose = world pose since the robot is at
the origin), ~robot_model, ~tcp_offset, ~rate
"""
import math
import threading

import numpy as np
import rospy
from gazebo_msgs.msg import LinkStates, ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from robot_chess.board_geometry import BoardGeometry
from robot_chess.move_inference import BLACK, EMPTY, WHITE
from robot_chess.piece_models import piece_sdf
from robot_chess.pieces import initial_square_models
from robot_chess_msgs.srv import ReplacePiece, ReplacePieceResponse


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class SimPieceManager(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.robot = rospy.get_param("~robot_model", "limo")
        self.tcp_offset = rospy.get_param("~tcp_offset", 0.06)
        self.grasp_radius = rospy.get_param("~grasp_radius", 0.015)
        self.lock = threading.Lock()
        self.models = {}            # model name -> Pose (all models)
        self.flange = None          # Pose of the robot's flange link
        self.held = None            # (model, offset below TCP)
        self.promotions = 0

        for srv in ("/gazebo/spawn_sdf_model", "/gazebo/delete_model",
                    "/gazebo/set_model_state"):
            rospy.wait_for_service(srv)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.occ_pub = rospy.Publisher("/chess_sim/occupancy", String, queue_size=1, latch=True)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.on_links, queue_size=1)

        self.reset_board()
        rospy.Service("~gripper/attach", Trigger, self.on_attach)
        rospy.Service("~gripper/release", Trigger, self.on_release)
        rospy.Service("~replace", ReplacePiece, self.on_replace)
        rospy.Service("~reset", Trigger, lambda _r: (self.reset_board(),
                                                     TriggerResponse(True, "reset"))[1])
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~rate", 30.0)), self.update_held)
        rospy.Timer(rospy.Duration(0.5), self.publish_occupancy)
        rospy.loginfo("sim piece manager ready")

    # ------------------------------------------------------------ state
    def on_models(self, msg):
        with self.lock:
            self.models = dict(zip(msg.name, msg.pose))

    def on_links(self, msg):
        name = "%s::joint6_flange" % self.robot
        if name in msg.name:
            self.flange = msg.pose[msg.name.index(name)]

    def tcp_world(self):
        f = self.flange
        if f is None:
            return None
        axis = quat_matrix(f.orientation)[:, 2]
        p = np.array([f.position.x, f.position.y, f.position.z])
        return p + self.tcp_offset * axis

    def pieces(self):
        with self.lock:
            return {n: p for n, p in self.models.items() if n.startswith(("white_", "black_"))}

    def board_xy(self, pose):
        """Board-frame (x, y) of a world pose (robot at the world origin)."""
        dx, dy = pose.position.x - self.g.board_x, pose.position.y - self.g.board_y
        c, s = math.cos(self.g.board_yaw), math.sin(self.g.board_yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def publish_occupancy(self, _event=None):
        occ = [EMPTY] * 64
        for name, pose in self.pieces().items():
            if self.held and self.held[0] == name:
                continue
            if abs(pose.position.z - self.g.board_z) > 0.005:
                continue
            sq = self.g.square_at_board(*self.board_xy(pose))
            if sq is not None:
                occ[sq] = WHITE if name.startswith("white_") else BLACK
        self.occ_pub.publish(String("".join(occ)))

    # ---------------------------------------------------------- spawning
    def _spawn(self, name, color, ptype, x, y, z):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        resp = self.spawn(model_name=name, model_xml=piece_sdf(name, color, ptype),
                          robot_namespace="", initial_pose=pose, reference_frame="world")
        if not resp.success:
            rospy.logerr("spawning %s failed: %s", name, resp.status_message)
        return resp.success

    def reset_board(self):
        self.held = None
        rospy.sleep(0.5)
        for name in list(self.pieces()):
            self.delete(model_name=name)
        for square, name in sorted(initial_square_models().items()):
            color, ptype, _ = name.split("_", 2)
            p = self.g.to_robot(*self.g.square_center_board(square))
            self._spawn(name, color, ptype, p[0], p[1], p[2])
        rospy.loginfo("32 pieces in the start position")

    # ----------------------------------------------------------- grasping
    def nearest_piece(self, point):
        best = None
        for name, pose in self.pieces().items():
            d = math.hypot(pose.position.x - point[0], pose.position.y - point[1])
            if best is None or d < best[0]:
                best = (d, name, pose)
        return best

    def update_held(self, _event):
        held = self.held
        tcp = self.tcp_world()
        if held is None or tcp is None:
            return
        st = ModelState()
        st.model_name = held[0]
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y = tcp[0], tcp[1]
        st.pose.position.z = tcp[2] - held[1]
        st.pose.orientation.w = 1.0
        self.state_pub.publish(st)

    def on_attach(self, _req):
        tcp = self.tcp_world()
        if tcp is None:
            return TriggerResponse(False, "gripper pose unknown")
        best = self.nearest_piece(tcp)
        if best is None or best[0] > self.grasp_radius:
            return TriggerResponse(False, "no piece between the fingers (nearest %.3f m)"
                                   % (best[0] if best else -1))
        d, name, pose = best
        self.held = (name, tcp[2] - pose.position.z)
        rospy.loginfo("gripper holds %s (%.1f mm off centre)", name, d * 1000)
        return TriggerResponse(True, name)

    def on_release(self, _req):
        held, self.held = self.held, None
        if held is None:
            return TriggerResponse(True, "nothing held")
        rospy.sleep(0.1)
        tcp = self.tcp_world()
        st = ModelState()
        st.model_name = held[0]
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y = tcp[0], tcp[1]
        st.pose.position.z = self.g.board_z      # put it on the table / board surface
        st.pose.orientation.w = 1.0
        resp = self.set_state(model_state=st)
        rospy.loginfo("released %s", held[0])
        return TriggerResponse(resp.success, held[0])

    def on_replace(self, req):
        tcp = self.tcp_world()
        best = self.nearest_piece(tcp) if tcp is not None else None
        if best is None or best[0] > self.grasp_radius:
            return ReplacePieceResponse(False, "no piece at the gripper")
        _, name, pose = best
        self.delete(model_name=name)
        self.promotions += 1
        new = "%s_%s_promoted_%d" % (req.color, req.piece_type, self.promotions)
        ok = self._spawn(new, req.color, req.piece_type,
                         pose.position.x, pose.position.y, pose.position.z)
        return ReplacePieceResponse(ok, new)


if __name__ == "__main__":
    rospy.init_node("chess_sim")
    SimPieceManager()
    rospy.spin()
