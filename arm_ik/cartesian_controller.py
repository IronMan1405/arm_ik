import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState
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
        self.current_exec_goal = None
        self.current_joint_state = None

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

        self.get_logger().info('Controls: [SPACE]=E-SYOP | [h]=HOME | [r]=RESET')

        # self.move_linear(0.0, 0.0, 0.05)

    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def get_current_joint_state(self):
        return self.current_joint_state
    
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
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def trigger_estop(self):
        if self.estop_active:
            return
        
        self.estop_active = True
        self.get_logger().error("!!! EMERGENCY STOP ACTIVATED !!!")

        if self.current_exec_goal:
            self.current_exec_goal.cancel_goal_async()

    def reset_estop(self):
        self.estop_active = False
        self.get_logger().info('E-STOP cleared')

    def get_current_ee_pose(self):
        if self.current_joint_state is None:
            self.get_logger().error('No joint state yet')
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['end_effector_link']
        req.robot_state = RobotState()
        req.robot_state.joint_state = self.current_joint_state

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        if not res or not res.pose_stamped:
            self.get_logger().error("FK Failed")
            return None
        
        return res.pose_stamped[0].pose
    

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
        traj.points.append(home)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj

        self.get_logger().info("Moving to HOME pose...")

        future = self.exec_client.send_goal_async(goal)
        future.add_done_callback(self.store_exec_goal)

    def store_exec_goal(self, future):
        self.current_exec_goal = future.result()

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
        target_pose.orientation = start_pose.orientation

        if not self.within_workspace(target_pose):
            self.get_logger().error(
                f'''Target pose out of workspace
                x={target_pose.position.x:.2f}
                y={target_pose.position.y:.2f}
                z={target_pose.position.z:.2f}'''
            )

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

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution

        self.get_logger().info('Executing Cartesian Trajectory...')
        self.exec_client.send_goal_async(goal)

    # def fk_response_callback(self, future):
    #     res = future.result()

    #     if not res or not res.pose_stamped:
    #         self.get_logger().error('FK failed')
    #         return
        
    #     self.curremt_ee_pose = res.pose_stamped[0].pose


    # def cartesian_response_callback(self, future):
    #     if self.estop_active:
    #         return
        
    #     res = future.result()

    #     self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

    #     if res.fraction < 0.95:
    #         self.get_logger().error('Cartesian path incomplete - aborting')
    #         rclpy.shutdown()
    #         return
        
    #     self.scale_trajectory(res.solution.joint_trajectory)

    #     exec_goal = ExecuteTrajectory.Goal()
    #     exec_goal.trajectory = res.solution

    #     self.get_logger().info('Executing Cartesian Trajectory...')
    #     self.exec_client.send_goal_async(exec_goal)


def main():
    rclpy.init()
    node = CartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()