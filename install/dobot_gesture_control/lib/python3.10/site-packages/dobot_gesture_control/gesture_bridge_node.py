import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import socket
import threading
import json
import math

def is_within_workspace(x, y, z):
    # 중심에서의 거리 계산
    distance = math.sqrt(x**2 + y**2)
    
    # 워크스페이스 범위 설정
    max_radius = 320  # mm
    min_z = 0         # mm
    max_z = 150       # mm

    # 거리와 Z축 범위 확인
    if distance <= max_radius and min_z <= z <= max_z:
        return True
    else:
        return False

class GestureTCPBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_tcp_bridge_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.hand_pub = self.create_publisher(Point, '/hand_position', 10)
        self.get_logger().info("Gesture TCP Bridge Node started.")

        # TCP 서버 시작 (백그라운드 스레드)
        self.server_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.server_thread.start()

    def start_tcp_server(self):
        HOST = '192.168.110.121'
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

                            # 좌표 메시지인지 제스처 문자열인지 분기 처리
                            if message.startswith('{') and 'x' in message and 'y' in message:
                                pos = json.loads(message)
                                point = Point()
                                point.y = 320- ((float(pos['y']) - 0.03) * 600) / 0.9
                                point.x = 300 - ((float(pos['x']) -0.065) * 420) / 0.865
                                point.z = 100.0
                                # 워크스페이스 체크
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