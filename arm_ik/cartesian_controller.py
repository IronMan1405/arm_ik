import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import RobotState, RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_srvs.srv import Trigger

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

        # self.estop_active = False
        self.current_joint_state = None

        self.exec_done_event = threading.Event()
        self.exec_result = None
        # self.current_exec_goal = None

        self.fk_event = threading.Event()
        self.last_fk_pose = None

        # self.motion_allowed = True

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.get_logger().info('Waiting for MoveIt services...')
        self.cartesian_client.wait_for_service()
        self.fk_client.wait_for_service()
        self.exec_client.wait_for_server()
        self.get_logger().info('Core MoveIt interfaces ready')

        # self.stop_client = self.create_client(Trigger, '/move_group/stop')
        # self.stop_client.wait_for_service()

        self.workspace = {
            'x': (-0.3575, 0.3575),
            'y': (-0.3575, 0.3575),
            'z': (0.095331, 0.4525),
        }

        self.velocity_scale = 0.3
        self.accel_scale = 0.2

        self.cartesian_event = threading.Event()
        self.last_cartesian_res = None

        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        # self.get_logger().info('Controls: [SPACE]=E-STOP | [h]=HOME | [r]=RESET | [t]=TEST')
        self.get_logger().info('Controls: [h]=HOME | [t]=TEST')

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            while rclpy.ok():
                key = sys.stdin.read(1)

                # if key == ' ':
                #     self.trigger_estop()
                # elif key == 'h':
                if key == 'h':
                    self.go_home()
                # elif key == 'r':
                #     self.reset_estop()
                elif key == 't':
                    # self.test_cartesian_sequence()
                    # self.test_segmented_cartesian(0.17064022,-0.02162209,-0.069)
                    # self.test_segmented_cartesian(dz=-0.069)
                    myz = 0.40011-0.4525
                    self.test_segmented_cartesian(dz=myz)
                    # self.test_segmented_cartesian(dx=0.17064022)
                    # self.test_segmented_cartesian(dy=-0.02162209)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    # def trigger_estop(self):
    #     self.estop_active = True
    #     # self.motion_allowed = False
        
    #     self.get_logger().error("!!! EMERGENCY STOP ACTIVATED !!!")

    #     if self.current_exec_goal:
    #         self.current_exec_goal.cancel_goal_async()

    #     # if self.stop_client.service_is_ready():
    #     #     req = Trigger.Request()
    #     #     self.stop_client.call_async(req)
    #     # else:
    #     if not self.stop_client.service_is_ready:
    #         self.get_logger().error("MoveIt stop service not ready yet - retrying in 0.5s")
    #         self.create_timer(0.5, self._retry_stop_service)
    #         return
        
    #     req = Trigger.Request()
    #     self.stop_client.call_async(req)
        
    #     # self.exec_done_event.set()

    # def _retry_stop_service(self):
    #     if self.stop_client.service_is_ready():
    #         self.get_logger("Moveit stop service is now ready - sending stop command")
    #         req = Trigger.Request()
    #         self.stop_client.call_async(req)
    #         return True
    #     else:
    #         self.get_logger().warn("still waiting for /move_group/stop...")
    #         return False

    def execute_trajectory(self, traj: JointTrajectory):
        # if not self.motion_allowed:
        #     self.get_logger().error("Motion locked due to E-STOP")
        #     return False
        
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
        # if self.estop_active:
        # if not self.motion_allowed:
        #     self.get_logger().error('Cannot go home - E-STOP active')
        #     return

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
        req.robot_state.is_diff = False

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
        # if self.estop_active:
        # if not self.motion_allowed:
        #     self.get_logger().error('E-STOP active, aborting sequence')
        #     return
        
        # self.move_linear(0.1, 0.0, 0.0)
        # self.wait_for_execution()

        self.move_linear(0.0, 0.0, -0.1)
        self.wait_for_execution()

        self.move_linear(-0.1, 0.0, 0.0)
        self.wait_for_execution()

    def get_current_joint_state(self):
        return self.current_joint_state

    # def reset_estop(self):
    #     self.estop_active = False
    #     self.motion_allowed = True
    #     self.get_logger().info('E-STOP cleared')

    def within_workspace(self, pose: Pose) -> bool:
        x, y, z = pose.position.x, pose.position.y, pose.position.z

        if not (self.workspace['x'][0] <= x <= self.workspace['x'][1]):
            return False
        if not (self.workspace['y'][0] <= y <= self.workspace['y'][1]):
            return False
        if not (self.workspace['z'][0] <= z <= self.workspace['z'][1]):
            return False
        
        return True
    
    def make_linear_waypoints(self, start: Pose, target: Pose, n_points: int = 20):
        waypoints = []

        for i in range(n_points + 1):
            alpha = i / n_points

            p = Pose()
            p.position.x = start.position.x + alpha * (target.position.x - start.position.x)
            p.position.y = start.position.y + alpha * (target.position.y - start.position.y)
            p.position.z = start.position.z + alpha * (target.position.z - start.position.z)

            p.orientation = start.orientation

            waypoints.append(p)

        return waypoints

    def scale_trajectory(self, traj: JointTrajectory):
        for point in traj.points:
            if point.velocities:
                point.velocities = [v * self.velocity_scale for v in point.velocities]

            if point.accelerations:
                point.accelerations = [a * self.accel_scale for a in point.accelerations]
    
    def move_linear(self, dx, dy, dz):
        # if not self.motion_allowed:
        #     self.get_logger().error("Motion locked due to E-STOP")
        #     return
        
        # if self.estop_active:
        #     return

        start_pose = self.get_current_ee_pose()
        if start_pose is None:
            return
        
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x + dx
        target_pose.position.y = start_pose.position.y + dy
        target_pose.position.z = start_pose.position.z + dz

        if not self.within_workspace(target_pose):
            self.get_logger().error("Target pose out of workspace")
            return

        req = GetCartesianPath.Request()
        req.group_name = 'arm'
        req.link_name = 'end_effector_link'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        req.start_state = RobotState()
        req.start_state.joint_state = self.current_joint_state
        req.waypoints = self.make_linear_waypoints(start_pose, target_pose, 25)

        self.get_logger().info(f'Planning Cartesian move: dx={dx}, dy={dy}, dz={dz}')

        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self._on_cartesian_response)
        # rclpy.spin_until_future_complete(self, future)
        # res = future.result()
        # self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

        # if res.fraction < 0.95:
        #     self.get_logger().error('Cartesian path incomplete - aborting')
        #     return

        # self.scale_trajectory(res.solution.joint_trajectory)

        # self.execute_trajectory(res.solution.joint_trajectory)

    def _on_fk_response(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"FK service failed: {e}")
            self.fk_event.set()
            return

        if not res or not res.pose_stamped:
            self.get_logger().error('FK failed')
            self.fk_event.set()
            return
    
        self.last_fk_pose = res.pose_stamped[0].pose
        self.fk_event.set()

    def _on_cartesian_response(self, future):
        # if self.estop_active:
        #     return
        
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Cartesian Service failed: {e}")
            return

        self.get_logger().info(f'Cartesian fraction: {res.fraction:.2f}')

        if res.fraction < 0.95:
            self.get_logger().error('Cartesian path incomplete - aborting')
            # rclpy.shutdown()
            return
        
        self.scale_trajectory(res.solution.joint_trajectory)
        self.execute_trajectory(res.solution.joint_trajectory)


        self.get_logger().info('Executing Cartesian Trajectory...')

    def execute_cartesian_segmented(self, target_pose, step_size=0.02, eef_step=0.005, jump_threshold=0.0, min_fraction=0.05, pos_tolerance=0.005):
        while rclpy.ok():
            start_pose = self.get_current_ee_pose()
            if start_pose is None:
                return False

            dx = target_pose.position.x - start_pose.position.x
            dy = target_pose.position.y - start_pose.position.y
            dz = target_pose.position.z - start_pose.position.z

            dist = (dx**2 + dy**2 + dz**2) ** 0.5\
            
            if dist < pos_tolerance:
                self.get_logger().info("Target reached")
                return True

            # num_segments = max(1, int(dist/step_size))
            scale = min(1.0, step_size / dist)

            step_pose = Pose()
            step_pose.position.x = start_pose.position.x + dx * scale
            step_pose.position.y = start_pose.position.y + dy * scale
            step_pose.position.z = start_pose.position.z + dz * scale

            # step_pose.orientation = start_pose.orientation

            # self.get_logger().info(f"Segmented Cartesian move: {num_segments} segments")\
            
            req = GetCartesianPath.Request()
            req.group_name = 'arm'
            req.link_name = 'end_effector_link'
            req.max_step = eef_step
            req.jump_threshold = jump_threshold
            req.avoid_collisions = True

            req.start_state = RobotState()
            req.start_state.joint_state = self.current_joint_state
            req.waypoints = [step_pose]

            future = self.cartesian_client.call_async(req)
            
            while not future.done():
                pass

            res = future.result()
            if res is None:
                self.get_logger().error("Cartesian service returned None")
                return False
            
            self.get_logger().info(f"Cartesian fraction = {res.fraction:.2f}")

            if res.fraction < min_fraction:
                self.get_logger().warn("Low fraction but non-zer -> executing partial path")

            if res.fraction == 0.0:
                self.get_logger().error("zero fraction -> motion impossible")

            self.scale_trajectory(res.solution.joint_trajectory)
            success = self.execute_trajectory(res.solution.joint_trajectory)

            if not success:
                self.get_logger().error("execution failed")
                return False


            # for i in range(1, num_segments + 1):
            #     ratio = i / num_segments

            #     intermediate_pose = Pose()
            #     intermediate_pose.position.x = start_pose.position.x + dx * ratio
            #     intermediate_pose.position.y = start_pose.position.y + dy * ratio
            #     intermediate_pose.position.z = start_pose.position.z + dz * ratio
            #     intermediate_pose.orientation = start_pose.orientation
                
            #     req = GetCartesianPath.Request()
            #     req.group_name = 'arm'
            #     req.link_name = 'end_effector_link'
            #     req.max_step = eef_step
            #     req.jump_threshold = jump_threshold
            #     req.avoid_collisions = True

            #     req.start_state = RobotState()
            #     req.start_state.joint_state = self.current_joint_state
            #     req.waypoints = [intermediate_pose]

            #     self.cartesian_event.clear()
            #     future = self.cartesian_client.call_async(req)
            #     future.add_done_callback(self._on_segment_cartesian_response)

            #     # res = future.result()
            #     if not self.cartesian_event.wait(2.0):
            #         self.get_logger().error(f"Segment {i}: Cartesian service failed")
            #         return False
                
            #     res = self.last_cartesian_res

            #     if res is None:
            #         self.get_logger().error(f"Segment {i}: Cartesian service failed - no result")
            #         return False

            #     self.get_logger().info(f"[Segment {i}/{num_segments}] Cartesian fraction: {res.fraction:.2f}")

            #     if res.fraction == 0.0:
            #         self.get_logger().error(f"Cartesian planning failed at segment {i}")
            #         return False
                
            #     self.scale_trajectory(res.solution.joint_trajectory)
            #     self.execute_trajectory(res.solution.joint_trajectory)
            #     self.wait_for_execution()

            #     start_pose = self.get_current_ee_pose()
            #     if start_pose is None:
            #         return False

            self.get_logger().info("Segmented Cartesian motion complete!")
            return True
    
    def _on_segment_cartesian_response(self, future):
        try:
            self.last_cartesian_res = future.result()
        except Exception as e:
            self.get_logger().error(f"Cartesian service failed: {e}")
            self.last_cartesian_res = None
        finally:
            self.cartesian_event.set()

    def test_segmented_cartesian(self, dx=None, dy=None, dz=None):
        start_pose = self.get_current_ee_pose()
        if start_pose is None:
            return

        target = Pose()

        if dx is not None:
            target.position.x = start_pose.position.x + dx
        else:
            target.position.x = start_pose.position.x
        
        if dy is not None:
            target.position.y = start_pose.position.y + dy
        else:
            target.position.y = start_pose.position.y

        if dz is not None:
            target.position.z = start_pose.position.z + dz
        else:
            target.position.z = start_pose.position.z

        # target.orientation = start_pose.orientation

        print(f"dx={dx} dy={dy} dz={dz}")
        
        self.execute_cartesian_segmented(target)


def main():
    rclpy.init()
    node = CartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()