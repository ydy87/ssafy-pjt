import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class GestureTCPBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_tcp_bridge_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.get_logger().info("Gesture TCP Bridge Node started.")

        # TCP 서버 시작 (백그라운드 스레드)
        self.server_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.server_thread.start()

    def start_tcp_server(self):
        HOST = '192.168.110.116'
        PORT = 8765

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # 포트 재사용 허용
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
                            msg = String()
                            msg.data = message
                            self.publisher_.publish(msg)
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
