import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

X_MIN, X_MAX = -1.65, 1.65
Y_MIN, Y_MAX = -1.65, 1.65
Z_MIN, Z_MAX = 0.402, 2.05

def is_reachable(p):
    return (
        X_MIN <= p.x <= X_MAX and
        Y_MIN <= p.y <= Y_MAX and
        Z_MIN <= p.z <= Z_MAX
    )

class IKMoveClient(Node):
    def __init__(self):
        super().__init__('ik_move_client')

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.client.wait_for_server()

        self.send_pose_goal()

    def send_pose_goal(self):
        goal = MoveGroup.Goal()

        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = 0.45
        pose.pose.position.y = 0.33
        pose.pose.position.z = 1.9
        pose.pose.orientation.w = 1.0

        if not is_reachable(pose.pose.position):
            self.get_logger().error('Target pose is outside reachable workspace')
            return

        pose_constraint = PositionConstraint()
        pose_constraint.header = pose.header
        pose_constraint.link_name = 'end_effector_link'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]

        pose_constraint.constraint_region.primitives.append(primitive)
        pose_constraint.constraint_region.primitive_poses.append(pose.pose)
        pose_constraint.weight = 1.0

        # ori_constraint = OrientationConstraint()
        # ori_constraint.header = pose.header
        # ori_constraint.link_name = 'end_effector_link'
        # ori_constraint.orientation = pose.pose.orientation
        # ori_constraint.absolute_x_axis_tolerance = 0.1
        # ori_constraint.absolute_y_axis_tolerance = 0.1
        # ori_constraint.absolute_z_axis_tolerance = 0.1
        # ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pose_constraint)
        # constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('Sending IK + planning goal...')
        # self._action_client.send_goal_async(goal)
        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        pass

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Execution Result: {result.error_code.val}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = IKMoveClient()
    rclpy.spin(node)
    # rclpy.shutdown()

if __name__ == "__main__":
    main()