import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from dobot_msgs.action import PointToPoint
from rclpy.task import Future

class HandPositionController(Node):
    def __init__(self):
        super().__init__('hand_position_controller')

        self.subscription = self.create_subscription(
            Point,
            '/hand_position',
            self.listener_callback,
            10)

        self._action_client = ActionClient(self, PointToPoint, 'PTP_action')
        self.current_pose = [150.0, 0.0, 100.0, 0.0]
        self.is_moving = False

        self.get_logger().info('Hand position controller started.')

    def listener_callback(self, msg: Point):
        if self.is_moving:
            self.get_logger().info('Dobot is moving. Ignoring current hand position.')
            return

        self.current_pose[0] = msg.x
        self.current_pose[1] = msg.y
        self.current_pose[2] = msg.z  # 고정값이긴 하지만 확장 가능

        self.send_goal(self.current_pose.copy(), mode=1)

    def send_goal(self, target, mode):
        self.is_moving = True
        self.get_logger().info(f'Sending goal: {target}')

        self._action_client.wait_for_server()
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = target
        goal_msg.motion_type = mode

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
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

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.current_pose}')


def main(args=None):
    rclpy.init(args=args)
    node = HandPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
