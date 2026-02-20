import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math, time, heapq
from enum import Enum, auto

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

class ArmState(Enum):
    INITIALIZING = auto()
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    ERROR = auto()

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
        self.create_subscription(Bool, '/arm/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.jointstate_callback, 10)

        self.traj_pub = self.create_publisher(JointTrajectory, '/arm/joint_trajectory', 10)
        self.state_pub = self.create_publisher(String, "/arm/execution_state", 10)
        self.stop_traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.emergency = False

        self.state = ArmState.INITIALIZING
        self.publish_state()
        self.command_queue = []
        self.command_counter = 0
        self.current_goal_id = 0
        self.create_timer(0.05, self.process_queue)

        self.latest_joint_state = None

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

        self.init_joint_names =["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.init_joint_positions = [
            math.radians(0),
            math.radians(20),
            math.radians(-20),
            math.radians(0),
            math.radians(0),
            math.radians(0)
        ]

        self.get_logger().info("IK Move Node ready")

        self.wait_for_initial_state()
        self.move_to_home()

    def enqueue_command(self, priority, cmd_type, data):
        self.command_counter += 1

        if self.state == ArmState.EXECUTING and priority == 1:
            if hasattr(self, "active_goal_handle"):
                self.get_logger().info("Preempting current motion")
                self.active_goal_handle.cancel_goal_async()
                # self.preempted = True

            self.state = ArmState.IDLE
            self.publish_state()
        
        heapq.heappush(self.command_queue, (priority, self.command_counter, cmd_type, data))

    def process_queue(self):
        if self.emergency:
            return

        if self.state != ArmState.IDLE:
            return
        if not self.command_queue:
            return

        priority, _, cmd_type, data = heapq.heappop(self.command_queue) 

        self.state = ArmState.PLANNING
        self.publish_state()

        if cmd_type == "pose":
            self.send_ik_goal(data)
        if cmd_type == "joint":
            self.joint_pose_to_fk_and_plan(data)

    def jointstate_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def wait_for_initial_state(self):
        self.get_logger().info("Waiting for joint states...")

        while rclpy.ok() and self.latest_joint_state is None:
            rclpy.spin_once(self)
        
        self.get_logger().info("Intial joint state recieved.")

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def publish_current_state_as_trajectory(self):
        if not hasattr(self, 'latest_joint_state'):
            return
        
        traj = JointTrajectory()
        traj.joint_names = self.latest_joint_state.name
        
        point = JointTrajectoryPoint()
        point.positions = list(self.latest_joint_state.position)
        point.time_from_start.sec = 0
        
        traj.points.append(point)
        self.stop_traj_pub.publish(traj)

    def emergency_callback(self, msg: Bool):
        self.emergency = msg.data

        if self.emergency:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED! Cancelling active trajectory...")

            if hasattr(self, "active_goal_handle"):
                self.active_goal_handle.cancel_goal_async()

            self.command_queue.clear()

            self.publish_current_state_as_trajectory()
            if hasattr(self, 'latest_joint_state'):
                self.freeze_rviz()
                self.get_logger().info('RViz frozen at current joints')
        
            self.state = ArmState.IDLE
            self.publish_state()


    # freezes rviz by sending current joint state as target end
    # def freeze_rviz(self):
    #     traj = JointTrajectory()
    #     traj.joint_names = self.latest_joint_state.name
        
    #     point = JointTrajectoryPoint()
    #     point.positions = self.latest_joint_state.position
    #     point.time_from_start.sec = 0
        
    #     traj.points.append(point)
    #     self.traj_pub.publish(traj)

    def delta_callback(self, msg: Vector3):
        if self.target_pose is None:
            return
        
        self.target_pose.pose.position.x += msg.x
        self.target_pose.pose.position.y += msg.y
        self.target_pose.pose.position.z += msg.z

        if not is_plausible_target(self.target_pose.pose.position):
            self.get_logger().warn("Target outside workspace, ignoring")
            return
        
        import copy
        self.enqueue_command(1, "pose", copy.deepcopy(self.target_pose))

    def orientation_callback(self, msg: Bool):
        self.orientation_enabled = msg.data

        state = "ENABLED" if self.orientation_enabled else "DISABLED"
        self.get_logger().info(f"Orientation constraint {state}")

    def named_pose_callback(self, msg: String):
        name = msg.data
        
        if name in self.named_poses:
            import copy
            self.target_pose = copy.deepcopy(self.named_poses[msg.data])
            self.enqueue_command(2, "pose", self.target_pose)
            return
        
        if name in self.named_joint_poses:
            self.enqueue_command(2, "joint", self.named_joint_poses[name])
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
        if self.emergency:
            return
        
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
        if self.emergency:
            return
        
        res = future.result()

        if not res or not res.pose_stamped:
            self.get_logger().warn("FK failed")

            self.state = ArmState.ERROR
            self.publish_state()

            self.state = ArmState.IDLE
            self.publish_state()
            return
        
        pose = res.pose_stamped[0]

        if not is_plausible_target(pose.pose.position):
            self.get_logger().warn("FK pose outside workspace, ignoring")

            self.state = ArmState.ERROR
            self.publish_state()

            self.state = ArmState.IDLE
            self.publish_state()
            return
        
        self.enqueue_command(1, "pose", pose)

        self.state = ArmState.IDLE
        self.publish_state()
    
    def send_ik_goal(self, pose: PoseStamped):
        if self.emergency:
            return
        
        self.current_goal_id += 1
        goal_id = self.current_goal_id

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
        send_future.add_done_callback(lambda future: self.goal_response_callback(future, goal_id))

    def goal_response_callback(self, future, goal_id):
        if self.emergency:
            return
        
        goal_handle = future.result()
        
        self.active_goal_handle = goal_handle

        if not goal_handle.accepted:
            self.state = ArmState.ERROR
            self.publish_state()
            self.state = ArmState.IDLE
            self.publish_state()
            self.get_logger().warn("IK goal rejected")
            return

        self.state = ArmState.EXECUTING
        self.publish_state()
        self.get_logger().info("IK goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self.ik_result_callback(future, goal_id))

    def ik_result_callback(self, future, goal_id):
        if self.emergency:
            return
        
        if goal_id != self.current_goal_id:
            self.get_logger().info("Ignoring old goal result")
            return
        
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.state = ArmState.ERROR
            self.publish_state()

            self.state = ArmState.IDLE
            self.publish_state()

            self.get_logger().warn(f"IK planning failed with code {result.error_code.val}")

            return
        
        trajectory = result.planned_trajectory.joint_trajectory

        if not trajectory.points:
            self.state = ArmState.ERROR
            self.publish_state()

            self.state = ArmState.IDLE
            self.publish_state()

            self.get_logger().warn("empty trajectory")

            return

        self.state = ArmState.IDLE
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
    
    def move_to_home(self):
        if self.emergency:
            self.get_logger().warn("Emergency active. skipping home motion.")
            return
        
        self.current_goal_id += 1
        goal_id = self.current_goal_id

        goal = MoveGroup.Goal()

        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        goal.request.start_state.joint_state = self.latest_joint_state

        constraints = Constraints()

        for name, position in zip(self.init_joint_names, self.init_joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info("Moving to Init Home pose...")

        future = self.client.send_goal_async(goal)
        future.add_done_callback(lambda duture: self.goal_response_callback(future, goal_id))

def main():
    rclpy.init()
    node = IKMoveNode()
    rclpy.spin(node)