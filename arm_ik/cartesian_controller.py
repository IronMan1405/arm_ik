import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState
from moveit_msgs.action import ExecuteTrajectory


class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')

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

        self.move_linear(0.0, 0.0, 0.05)

    def within_workspace(self, pose: Pose) -> bool:
        x, y, z = pose.position.x, pose.position.y, pose.position.z

        if not (self.workspace['x'][0] <= x <= self.workspace['x'][1]):
            return False
        if not (self.workspace['y'][0] <= y <= self.workspace['y'][1]):
            return False
        if not (self.workspace['z'][0] <= z <= self.workspace['z'][1]):
            return False
        
        return True

    def get_current_ee_pose(self):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['end_effector_link']
        req.robot_state = RobotState()

        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        if not res or not res.pose_stamped:
            self.get_logger().error("FK Failed")
            return None
        
        return res.pose_stamped[0].pose
    
    def move_linear(self, dx, dy, dz):
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
        req.waypoints = [start_pose, target_pose]

        self.get_logger().info(f'Planning Cartesian move: dx={dx}, dy={dy}, dz={dz}')

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.cartesian_response_callback)

    def cartesian_response_callback(self, future):
        res = future.result()

        self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

        if res.fraction < 0.95:
            self.get_logger().error('Cartesian path incomplete - aborting')
            rclpy.shutdown()
            return
        
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = res.solution

        self.get_logger().info('Executing Cartesian Trajectory...')
        self.exec_client.send_goal_async(exec_goal)


def main():
    rclpy.init()
    node = CartesianController()
    rclpy.spin(node)

if __name__ == "__main__":
    main()