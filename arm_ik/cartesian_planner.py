from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

class CartesianPlanner:
    def __init__(self, node, group_name, ee_link, ref_frame = "world"):
        self.node = node

        self.group_name = group_name
        self.ee_link = ee_link
        self.reference_frame = ref_frame

        self.ee_step = 0.005
        self.jump_threshold = 0.0
        self.min_fraction = 0.95
        self.avoid_collisions = True

        self.segment_size = 0.02
        self.position_tolerance = 0.005

        self.cartesian_client = self.node.create_client(GetCartesianPath, '/compute_cartesian_path')

        self.node.get_logger().info("CartesianPlanner initialized.")

    def generate_waypoints(self, start_pose: Pose, end_pose: Pose, n_points = 15):
        """Generate intermediate Cartesian waypoints."""
        
        waypoints = []
        
        for i in range(1, n_points + 1):
            alpha = i / n_points

            p = Pose()
            p.position.x = start_pose.position.x + alpha * (end_pose.position.x - start_pose.position.x)
            p.position.y = start_pose.position.y + alpha * (end_pose.position.y - start_pose.position.y)
            p.position.z = start_pose.position.z + alpha * (end_pose.position.z - start_pose.position.z)

            p.orientation = start_pose.orientation

            waypoints.append(p)

        return waypoints

    def plan_cartesian_path(self, start_state, waypoints, callback):
        """Request Cartesian path planning asynchronously."""
        
        if not self.cartesian_client.service_is_ready(timeout_sec=0.0):
            self.node.get_logger().error("Cartesian path service is unavailable.")
            callback(None, "Service unavailable")
            return

        if not waypoints:
            callback(None, "No waypoints provided")
            return

        req = GetCartesianPath.Request()
        req.header.frame_id = self.reference_frame

        req.group_name = self.group_name
        req.link_name = self.ee_link

        req.max_step = self.ee_step
        req.jump_threshold = self.jump_threshold
        req.avoid_collisions = self.avoid_collisions

        req.start_state = start_state

        req.waypoints = waypoints

        future = self.cartesian_client.call_async(req)

        future.add_done_callback(lambda future: self._cartesian_response_callback(future, callback))

    def _cartesian_response_callback(self, future, callback):
        try:
            response = future.result()

            if response is None:
                callback(None, "Empty result from MoveIt")
                return
            
            callback(response, None)
    
        except Exception as e:
            self.node.get_logger().error(f"Cartesian Service failed: {e}")

            callback(None, str(e))


    def plan_segmented_path(self, target_pose, callback):
        """Plan a long Cartesian movement as smaller segments."""
        pass

    def validate_response(self, response):
        """Check whether MoveIt generated a sufficiently complete path."""
        if response is None:
            return False

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(f"MoveIt Cartesian planning failed. "
                f"Error code: {response.error_code.val}")
            return False

        fraction = response.fraction

        self.node.get_logger().info(f"Cartesian path completion: {fraction: .2f}")

        if fraction < self.min_fraction:
            self.node.get_logger().warning("Cartesian path is incomplete")
            return False

        if not response.solution.joint_trajectory.points:
            self.node.get_logger().error("Cartesian path contains not trajectory poitns")
            return False

        return True