import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration


class IKService(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.get_logger().info('IK Service node started successfully')


def main():
    rclpy.init()
    node = IKService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
