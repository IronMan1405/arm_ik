import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath

class CartesianMoveClient(Node):
    def __init__(self):
        super().__init__('cartesian_move_client')

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for Cartesian path service...')
        self.cartesian_client.wait_for_service()

        self.get_logger().info('Waiting for execute trajectory action...')
        self.exec_client.wait_for_server()

        self.send_cartesian_request()

    def send_cartesian_request(self):
        req = GetCartesianPath.Request()

        req.group_name = 'arm'
        req.link_name = 'end_effector_link'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        req.start_state = RobotState()

        p1 = Pose()
        p1.position.x = 0.0
        p1.position.y = 0.0
        p1.position.z = 2.05
        p1.orientation.w = 1.0


        # 0.01231; 7.0466e-05; 1.7935
        p2 = Pose()
        p2.position.x = 0.0
        p2.position.y = 0.0
        p2.position.z = 1.80
        p2.orientation.w = 1.0

        req.waypoints = [p1, p2]

        self.get_logger().info('Requesting Cartesian Path....')
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
    node = CartesianMoveClient()
    rclpy.spin(node)

if __name__ == "__main__":
    main()