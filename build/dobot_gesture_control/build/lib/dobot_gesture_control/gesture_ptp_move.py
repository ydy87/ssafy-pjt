import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import String
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

class GestureBasedPTPMove(Node):
    def __init__(self):
        super().__init__('gesture_ptp_move_node')

        # Action client: PTP 이동
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')

        # Service client: 진공 흡착
        self._suction_client = self.create_client(SuctionCupControl, 'dobot_suction_cup_service')

        # 제스처 명령 구독
        self.subscription = self.create_subscription(
            String,
            '/gesture_cmd',
            self.gesture_callback,
            10
        )

        # 상태 변수
        self.current_pose = [150.0, 0.0, 100.0, 0.0]  # 초기 위치
        self.step = 10.0  # 이동 간격 (mm)
        self.is_moving = False  # 이동 중 여부
        self.next_command = None  # 대기 중인 제스처

        self.get_logger().info('Gesture-based PTP move node (vacuum mode) started.')

    def gesture_callback(self, msg):
        code = msg.data.strip()
        self.get_logger().info(f'Gesture received: {code}')

        if self.is_moving:
            self.get_logger().warn('Dobot is moving, command delayed.')
            self.next_command = code
            return

        self.handle_gesture(code)

    def handle_gesture(self, code):
        if code == '1':
            self.current_pose[0] -= self.step
        elif code == '2':
            self.current_pose[0] += self.step
        elif code == '3':
            self.current_pose[1] -= self.step
        elif code == '4':
            self.current_pose[1] += self.step
        elif code == '5':
            self.current_pose[2] -= self.step
        elif code == '6':
            self.current_pose[2] += self.step
        elif code == '7':
            self.control_suction(True)
            return
        elif code == '8':
            self.control_suction(False)
            return
        else:
            self.get_logger().warn(f'Unknown gesture code: {code}')
            return

        self.send_goal(self.current_pose.copy(), mode=1)

    def send_goal(self, target, mode):
        self.is_moving = True
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
            self.is_moving = False
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle
        self._goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.is_moving = False

        # 이동 중에 들어온 명령이 있으면 실행
        if self.next_command:
            code = self.next_command
            self.next_command = None
            self.get_logger().info(f'Executing delayed command: {code}')
            self.handle_gesture(code)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.current_pose}')

    def control_suction(self, enable: bool):
        if not self._suction_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Suction service not available.')
            return

        request = SuctionCupControl.Request()
        request.enable_suction = enable

        future = self._suction_client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Suction cup {"ON" if enable else "OFF"}'))

def main(args=None):
    rclpy.init(args=args)
    node = GestureBasedPTPMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
