import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive

import sys, termios, tty, select

# not true reachability limits, they just filter out obviously impossible targets
X_MIN, X_MAX = -0.3575, 0.3575
Y_MIN, Y_MAX = -0.3575, 0.3575
Z_MIN, Z_MAX = 0.095331, 0.4525

def is_plausible_target(p):
    return (
        X_MIN <= p.x <= X_MAX and
        Y_MIN <= p.y <= Y_MAX and
        Z_MIN <= p.z <= Z_MAX
    )

class IKMoveNode(Node):
    def __init__(self):
        super().__init__('ik_move_node')

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.client.wait_for_server()

        self.create_subscription(Vector3, "/arm/teleop_delta", self.delta_callback, 10)
        self.create_subscription(Bool, "/arm/orientation_constraint", self.orientation_callback, 10)
        self.create_subscription(String, "/arm/named_pose", self.named_pose_callback, 10)

        self.target_pose = self.get_current_pose()
        self.orientation_enabled = False
        self.named_poses = {
            'home' : PoseStamped(header=self.make_header(),pose=self.make_pose(0.0032001789659261703, -1.795403477444779e-05, 0.4174376428127289))
        }

        self.get_logger().info("IK Move Node ready")

    def delta_callback(self, msg: Vector3):
        if self.target_pose is None:
            return
        
        self.target_pose.pose.position.x += msg.x
        self.target_pose.pose.position.y += msg.y
        self.target_pose.pose.position.z += msg.z

        if not is_plausible_target(self.target_pose.pose.position):
            self.get_logger().warn("Target outside workspace, ignoring")
            return
        
        self.send_ik_goal(self.target_pose)

    def orientation_callback(self, msg: Bool):
        self.orientation_enabled = msg.data

        state = "ENABLED" if self.orientation_enabled else "DISABLED"
        self.get_logger().info(f"Orientation constraint {state}")

    def named_pose_callback(self, msg: String):
        if msg.data not in self.named_poses:
            self.get_logger().warn(f"Unknown named pose: {msg.data}")
            return
        
        self.target_pose = self.named_poses[msg.data]
        self.send_ik_goal(self.target_pose)

    
    def send_ik_goal(self, pose: PoseStamped):
        goal = MoveGroup.Goal()

        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 1.0 # 0.3
        goal.request.max_acceleration_scaling_factor = 1.0 # 0.3

        pose_constraint = PositionConstraint()
        pose_constraint.header = pose.header
        pose_constraint.link_name = 'end_effector_link'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.03, 0.03, 0.03]

        pose_constraint.constraint_region.primitives.append(primitive)
        pose_constraint.constraint_region.primitive_poses.append(pose.pose)
        pose_constraint.weight = 1.0


        constraints = Constraints()
        constraints.position_constraints.append(pose_constraint)

        if self.orientation_enabled:
            ori_constraint = OrientationConstraint()
            ori_constraint.header = pose.header
            ori_constraint.link_name = 'end_effector_link'
            ori_constraint.orientation = pose.pose.orientation
            ori_constraint.absolute_x_axis_tolerance = 0.2
            ori_constraint.absolute_y_axis_tolerance = 0.2
            ori_constraint.absolute_z_axis_tolerance = 0.2
            ori_constraint.weight = 1.0
            constraints.orientation_constraints.append(ori_constraint)


        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('Sending IK + planning goal...')
        self.client.send_goal_async(goal)

    def get_current_pose(self):
        ps = PoseStamped()
        ps.header = self.make_header()
        ps.pose = self.make_pose(0.0032001789659261703, -1.795403477444779e-05, 0.4174376428127289)
        return ps

    def make_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.frame_id = 'base_link'
        return h
    
    def make_pose(self, x, y, z):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        return p

def main():
    rclpy.init()
    node = IKMoveNode()
    rclpy.spin(node)