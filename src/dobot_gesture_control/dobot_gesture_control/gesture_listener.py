import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import String
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SetEndEffectorGripper

class GestureBasedPTPMove(Node):
    def __init__(self):
        super().__init__('gesture_ptp_move_node')

        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self._gripper_client = self.create_client(SetEndEffectorGripper, 'SetEndEffectorGripper')

        self.subscription = self.create_subscription(
            String,
            '/gesture_cmd',
            self.gesture_callback,
            10
        )

        self.current_pose = [200.0, 0.0, 100.0, 0.0]  # 초기 위치
        self.step = 10.0  # 이동 간격 (mm)

        self.get_logger().info('Gesture-based PTP move node started.')

    def gesture_callback(self, msg):
        code = msg.data.strip()
        self.get_logger().info(f'Gesture received: {code}')

        if code == '1':
            self.current_pose[0] -= self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '2':
            self.current_pose[0] += self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '3':
            self.current_pose[1] -= self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '4':
            self.current_pose[1] += self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '5':
            self.current_pose[2] -= self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '6':
            self.current_pose[2] += self.step
            self.send_goal(self.current_pose.copy(), mode=1)
        elif code == '7':
              self.control_gripper(True)   # 진공 ON (집기)
        elif code == '8':
             self.control_gripper(False)  # 진공 OFF (놓기)

    def send_goal(self, target, mode):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode

        self.get_logger().info(f'Sending goal: {target}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected.')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted.')
        self._goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.current_pose}')

    def control_gripper(self, close: bool):
        if not self._gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Gripper service not available.')
            return
        request = SetEndEffectorGripper.Request()
        request.enable_ctrl = True
        request.gripper = 1.0 if close else 0.0
        future = self._gripper_client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Gripper {"closed" if close else "opened"}'))

def main(args=None):
    rclpy.init(args=args)
    node = GestureBasedPTPMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
