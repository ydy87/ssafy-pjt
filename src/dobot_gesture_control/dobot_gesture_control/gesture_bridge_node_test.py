import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import socket
import threading
import json
import math
import asyncio
import websockets

# WebSocket 핸들러
async def handler(websocket, path):
    async for message in websocket:
        print(f"Received: {message}")
        await websocket.send("ACK")

def start_websocket_server():
    async def run():
        async with websockets.serve(handler, '0.0.0.0', 8000):
            print("WebSocket server started on ws://0.0.0.0:8000")
            await asyncio.Future()  # run forever

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run())


# 워크스페이스 유효성 검사 함수
def is_within_workspace(x, y, z):
    distance = math.sqrt(x**2 + y**2)
    max_radius = 320  # mm
    min_z = 0         # mm
    max_z = 150       # mm
    return distance <= max_radius and min_z <= z <= max_z

# ROS2 노드 클래스
class GestureTCPBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_tcp_bridge_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.hand_pub = self.create_publisher(Point, '/hand_position', 10)
        self.error_pub = self.create_publisher(String, '/gesture_error', 10)
        self.get_logger().info("Gesture TCP Bridge Node started.")

        # TCP 서버 스레드 시작
        self.server_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.server_thread.start()

        # WebSocket 서버 스레드 시작
        self.websocket_thread = threading.Thread(target=start_websocket_server, daemon=True)
        self.websocket_thread.start()

    def start_tcp_server(self):
        HOST = '192.168.110.116'
        PORT = 8765

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((HOST, PORT))
                s.listen(1)
                self.get_logger().info(f"TCP server listening on {HOST}:{PORT}")
                conn, addr = s.accept()
                self.get_logger().info(f"Connected by {addr}")

                with conn:
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            self.get_logger().info("Connection closed by client.")
                            break
                        try:
                            message = data.decode('utf-8').strip()
                            self.get_logger().info(f"Received from client: {message}")

                            # 제스처 명령 or 좌표 메시지 분기
                            if message.startswith('{') and 'x' in message and 'y' in message:
                                pos = json.loads(message)
                                point = Point()
                                point.y = 320 - ((float(pos['y']) - 0.03) * 600) / 0.9
                                point.x = 300 - ((float(pos['x']) - 0.065) * 420) / 0.865
                                point.z = 100.0
                                if is_within_workspace(point.x, point.y, point.z):
                                    self.hand_pub.publish(point)
                                    self.get_logger().info(f"✅ Published /hand_position: {point}")
                                else:
                                    error_msg = f"❌ Out of workspace: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
                                    self.get_logger().error(error_msg)
                                    self.error_pub.publish(String(data=error_msg))
                            else:
                                msg = String()
                                msg.data = message
                                self.publisher_.publish(msg)
                                self.get_logger().info(f"Published /gesture_cmd: {msg.data}")
                        except Exception as e:
                            self.get_logger().error(f"Decoding/publish error: {e}")
            except OSError as e:
                self.get_logger().error(f"Socket error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    node = GestureTCPBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
