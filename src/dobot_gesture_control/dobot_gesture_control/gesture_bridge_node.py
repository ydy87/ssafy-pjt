import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets

class GestureBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_bridge_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.get_logger().info("Gesture WebSocket Bridge Node started.")

    async def websocket_handler(self, websocket, path):
        async for message in websocket:
            self.get_logger().info(f"Received from WebSocket: {message}")
            msg = String()
            msg.data = message.strip()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GestureBridgeNode()

    # 웹소켓 서버 실행 (localhost:8765)
    loop = asyncio.get_event_loop()
    start_server = websockets.serve(node.websocket_handler, "0.0.0.0", 8765)
    loop.run_until_complete(start_server)
    node.get_logger().info("WebSocket server listening on ws://0.0.0.0:8765")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
