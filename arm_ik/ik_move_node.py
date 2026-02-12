import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

import time

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

        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.get_logger().info('Waiting for FK service...')
        self.fk_client.wait_for_service()

        self.create_subscription(Vector3, "/arm/teleop_delta", self.delta_callback, 10)
        self.create_subscription(Bool, "/arm/orientation_constraint", self.orientation_callback, 10)
        self.create_subscription(String, "/arm/named_pose", self.named_pose_callback, 10)

        self.traj_pub = self.create_publisher(JointTrajectory, '/arm/joint_trajectory', 10)
        self.state_pub = self.create_publisher(String, "/arm/execution_state", 10)

        self.execution_state = "IDLE"
        self.publish_state()

        self.target_pose = self.get_current_pose()
        self.orientation_enabled = False
        
        self.named_poses = {
            'home' : PoseStamped(header=self.make_header(),pose=self.make_pose(0.0032001789659261703, -1.795403477444779e-05, 0.4174376428127289)),
        }

        self.named_joint_poses = {
            'zero' : {
                'joint1': 0.0,
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                },
            'front' : {
                'joint1': 0.0,
                'joint2': 0.35,
                'joint3': 0.7,
                'joint4': 0.5,
                'joint5': 0.0,
            },
            'back' : {
                'joint1': 0.0,
                'joint2': -0.35,
                'joint3': -0.7,
                'joint4': -0.5,
                'joint5': 0.0,
            }
        }

        self.get_logger().info("IK Move Node ready")

    def publish_state(self):
        msg = String()
        msg.data = self.execution_state
        self.state_pub.publish(msg)

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
        name = msg.data
        
        if name in self.named_poses:
            # self.target_pose = self.named_poses[msg.data]
            import copy
            self.target_pose = copy.deepcopy(self.named_poses[msg.data])
            self.send_ik_goal(self.target_pose)
            return
        
        if name in self.named_joint_poses:
            self.joint_pose_to_fk_and_plan(self.named_joint_poses[name])
            return
        
        if name not in self.named_poses and name not in self.named_joint_poses:
            self.get_logger().warn(f"Unknown named pose: {name}")
            return

    # def publish_joint_pose(self, joint_map):
    #     msg = JointState()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.name = list(joint_map.keys())
    #     msg.position = list(joint_map.values())

    #     self.joint_pub.publish(msg)
    #     self.get_logger().info(f"Published joint pose: {joint_map}")

    def joint_pose_to_fk_and_plan(self, joint_map):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['end_effector_link']
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(joint_map.keys())
        req.robot_state.joint_state.position = list(joint_map.values())
        req.robot_state.is_diff = False

        future = self.fk_client.call_async(req)
        future.add_done_callback(self.fk_callback)

    def fk_callback(self, future):
        res = future.result()

        if not res or not res.pose_stamped:
            self.get_logger().warn("FK failed")
            return
        
        pose = res.pose_stamped[0]

        if not is_plausible_target(pose.pose.position):
            self.get_logger().warn("FK pose outside workspace, ignoring")
            return
        
        self.send_ik_goal(pose)
    
    def send_ik_goal(self, pose: PoseStamped):
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = MoveGroup.Goal()

        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

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
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.execution_state = "FAILED"
            self.publish_state()
            self.get_logger().warn("IK goal rejected")
            return

        self.execution_state = "EXECUTING"
        self.publish_state()
        self.get_logger().info("IK goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.ik_result_callback)

    def ik_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.execution_state = "FAILED"
            self.publish_state()
            self.get_logger().warn(f"IK planning failed with code {result.error_code.val}")
            return
        
        trajectory = result.planned_trajectory.joint_trajectory

        if not trajectory.points:
            self.execution_state = "FAILED"
            self.publish_state()
            self.get_logger().warn("empty trajectory")
            return

        if not result.planned_trajectory.joint_trajectory.points:
            self.execution_state = "FAILED"
            self.publish_state()
            self.get_logger().info("Empty trajectory")
            return

        self.execution_state = "SUCCEEDED"
        self.publish_state()
        self.traj_pub.publish(trajectory)
        self.get_logger().info("Published joint trajectory")

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