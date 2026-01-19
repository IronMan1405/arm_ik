import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import threading, sys, termios, tty

HOME_JOINTS = {
    "joint1": 0.0,
    "joint2": -1.2,
    "joint3": 1.5,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
}

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')

        self.estop_active = False
        self.current_joint_state = None

        self.exec_done_event = threading.Event()
        self.exec_result = None
        self.current_exec_goal = None

        self.fk_event = threading.Event()
        self.last_fk_pose = None

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for MoveIt services...')
        self.cartesian_client.wait_for_service()
        self.fk_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('All MoveIt interfaces ready')

        self.workspace = {
            'x': (-1.65, 1.65),
            'y': (-1.65, 1.65),
            'z': (0.402, 2.05),
        }

        self.velocity_scale = 0.3
        self.accel_scale = 0.2

        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        self.get_logger().info('Controls: [SPACE]=E-STOP | [h]=HOME | [r]=RESET | [t]=TEST')

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            while rclpy.ok():
                key = sys.stdin.read(1)

                if key == ' ':
                    self.trigger_estop()
                elif key == 'h':
                    self.go_home()
                elif key == 'r':
                    self.reset_estop()
                elif key == 't':
                    self.test_cartesian_sequence()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def trigger_estop(self):        
        self.estop_active = True
        self.get_logger().error("!!! EMERGENCY STOP ACTIVATED !!!")

        if self.current_exec_goal:
            self.current_exec_goal.cancel_goal_async()

    def execute_trajectory(self, traj: JointTrajectory):
        self.exec_done_event.clear()

        goal = ExecuteTrajectory.Goal()

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        goal.trajectory = robot_traj
        
        future = self.exec_client.send_goal_async(goal)
        future.add_done_callback(self._on_exec_goal)

        self.exec_done_event.wait()

        return self.exec_result.error_code.val == 1
    
    def _on_exec_goal(self, future):
        gh = future.result()

        if not gh.accepted:
            self.exec_done_event.set()
            return
        
        self.current_exec_goal = gh
        gh.get_result_async().add_done_callback(self._on_exec_result)

    def _on_exec_result(self, future):
        self.exec_result = future.result().result
        self.exec_done_event.set()

    def go_home(self):
        if self.estop_active:
            self.get_logger().error('Cannot go home - E-STOP active')
            return

        if self.current_joint_state is None:
            self.get_logger().error('No joint state')
            return
        
        traj = JointTrajectory()
        traj.joint_names = list(HOME_JOINTS.keys())

        name_to_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

        start = JointTrajectoryPoint()
        start.positions = list(name_to_pos[i] for i in traj.joint_names)
        start.time_from_start.sec = 0
        traj.points.append(start)

        home = JointTrajectoryPoint()
        home.positions = list(HOME_JOINTS[j] for j in traj.joint_names)
        home.time_from_start.sec = 3
        
        traj.points = [start, home]

        self.get_logger().info("Moving to HOME pose...")
        self.execute_trajectory(traj)

    def get_current_ee_pose(self):
        if self.current_joint_state is None:
            self.get_logger().error('No joint state yet')
            return None
        
        self.fk_event.clear()

        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['end_effector_link']
        req.robot_state = RobotState()
        req.robot_state.joint_state = self.current_joint_state

        future = self.fk_client.call_async(req)
        future.add_done_callback(self._on_fk_response)

        if not self.fk_event.wait(1.0):
            self.get_logger().error("FK timeout")
            return None
        
        return self.last_fk_pose

    def wait_for_execution(self, timeout=10.0):
        finished = self.exec_done_event.wait(timeout)

        if not finished:
            self.get_logger().error("Execution timed out")
            return False
        
        self.exec_done_event.clear()

        if self.exec_result.error_code.val != 1:
            self.get_logger().error(f"Execution failed with code {self.exec_result.error_code.val}")
            return False
        
        return True
    
    def test_cartesian_sequence(self):
        if self.estop_active:
            self.get_logger().error('E-STOP active, aborting sequence')
            return
        
        self.move_linear(0.1, 0.0, 0.0)
        self.wait_for_execution()

        self.move_linear(0.0, 0.0, -0.1)
        self.wait_for_execution()

        self.move_linear(-0.1, 0.0, 0.0)
        self.wait_for_execution()

    def get_current_joint_state(self):
        return self.current_joint_state

    def reset_estop(self):
        self.estop_active = False
        self.get_logger().info('E-STOP cleared')

    def within_workspace(self, pose: Pose) -> bool:
        x, y, z = pose.position.x, pose.position.y, pose.position.z

        if not (self.workspace['x'][0] <= x <= self.workspace['x'][1]):
            return False
        if not (self.workspace['y'][0] <= y <= self.workspace['y'][1]):
            return False
        if not (self.workspace['z'][0] <= z <= self.workspace['z'][1]):
            return False
        
        return True

    def scale_trajectory(self, traj: JointTrajectory):
        for point in traj.points:
            if point.velocities:
                point.velocities = [v * self.velocity_scale for v in point.velocities]

            if point.accelerations:
                point.accelerations = [a * self.accel_scale for a in point.accelerations]
    
    def move_linear(self, dx, dy, dz):
        if self.estop_active:
            return

        start_pose = self.get_current_ee_pose()
        if start_pose is None:
            return
        
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x + dx
        target_pose.position.y = start_pose.position.y + dy
        target_pose.position.z = start_pose.position.z + dz

        if not self.within_workspace(target_pose):
            self.get_logger().error("Target pose out of workspace")

        req = GetCartesianPath.Request()
        req.group_name = 'arm'
        req.link_name = 'end_effector_link'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state
        req.waypoints = [start_pose, target_pose]

        self.get_logger().info(f'Planning Cartesian move: dx={dx}, dy={dy}, dz={dz}')

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

        if res.fraction < 0.95:
            self.get_logger().error('Cartesian path incomplete - aborting')
            return

        self.scale_trajectory(res.solution.joint_trajectory)

        self.execute_trajectory(res.solution.joint_trajectory)

    def _on_fk_response(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"FK service failed: {e}")
            self.fk_event.set()
            return

        if not res or not res.pose_stamped:
            self.get_logger().error('FK failed')
            return
    
        self.last_fk_pose = res.pose_stamped[0].pose
        self.fk_event.set()

    def cartesian_response_callback(self, future):
        if self.estop_active:
            return
        
        res = future.result()

        self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

        if res.fraction < 0.95:
            self.get_logger().error('Cartesian path incomplete - aborting')
            rclpy.shutdown()
            return
        
        self.scale_trajectory(res.solution.joint_trajectory)

        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        self.get_logger().info('Executing Cartesian Trajectory...')
        future = self.exec_client.send_goal_async(exec_goal)
        future.add_done_callback(self._on_exec_result)


def main():
    rclpy.init()
    node = CartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()