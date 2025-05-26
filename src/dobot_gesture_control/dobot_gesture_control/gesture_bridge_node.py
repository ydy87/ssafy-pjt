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
import functools

def deg(rad):
    return rad * 180.0 / math.pi

a2 = 135.0
a3 = 147.0
d1 = 54.65

def is_within_workspace(x, y, z, r=0):
    xy_radius = math.hypot(x, y)
    min_radius = abs(a2 - a3) + 10
    max_radius = a2 + a3 - 10
    if not (min_radius <= xy_radius <= max_radius):
        return False, False, f"Out of XY workspace ({min_radius:.1f} ~ {max_radius:.1f} mm), xy_radius={xy_radius:.1f}"
    min_z = 0 + d1
    max_z = 150 + d1
    if not (min_z <= z <= max_z):
        return False, False, f"Out of Z range ({min_z} ~ {max_z}), z={z}"
    theta1 = math.atan2(y, x)
    theta1_deg = deg(theta1)
    if not (-135 <= theta1_deg <= 135):
        return False, False, f"Theta1 (base) out of range, θ1={theta1_deg:.1f}"
    try:
        wx, wy = x, y
        d = math.hypot(wx, wy)
        cos_theta2 = (d**2 - a2**2 - a3**2) / (2 * a2 * a3)
        if not -1.0 <= cos_theta2 <= 1.0:
            return False, False, f"Elbow angle (theta2) impossible, cosθ2={cos_theta2:.3f}"
        theta2 = math.acos(cos_theta2)
        theta2_deg = deg(theta2)
        k1 = a2 + a3 * math.cos(theta2)
        k2 = a3 * math.sin(theta2)
        theta1_ik = math.atan2(wy, wx) - math.atan2(k2, k1)
        theta1_ik_deg = deg(theta1_ik)
        if not (0 <= theta2_deg <= 135):
            return False, False, f"Theta2 (elbow) out of range, θ2={theta2_deg:.1f}"
        if not (-135 <= theta1_ik_deg <= 135):
            return False, False, f"Theta1 (shoulder, ik) out of range, θ1={theta1_ik_deg:.1f}"
    except Exception as e:
        return False, False, f"IK math error: {e}"
    singular = (abs(theta2_deg) < 1.0) or (abs(theta2_deg - 180) < 1.0)
    if singular:
        return True, True, "Singularity: arm nearly fully stretched or folded."
    return True, False, "Valid & reachable"

class GestureTCPBridgeNode(Node):
    def __init__(self):
        super().__init__('gesture_tcp_bridge_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmd', 10)
        self.hand_pub = self.create_publisher(Point, '/hand_position', 10)
        self.error_pub = self.create_publisher(String, '/gesture_error', 10)
        self.ws_clients = set()
        self.websocket_loop = None      # 추가: 웹소켓 이벤트루프 저장
        self.get_logger().info("Gesture TCP Bridge Node started.")

        self.server_thread = threading.Thread(target=self.start_tcp_server, daemon=True)
        self.server_thread.start()

        ws_func = functools.partial(self.start_websocket_server)
        self.websocket_thread = threading.Thread(target=ws_func, daemon=True)
        self.websocket_thread.start()

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
                            self.handle_incoming_message(message)
                        except Exception as e:
                            self.get_logger().error(f"Decoding/publish error: {e}")
            except OSError as e:
                self.get_logger().error(f"Socket error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def handle_incoming_message(self, message):
        if message.startswith('{') and 'x' in message and 'y' in message:
            pos = json.loads(message)
            point = Point()
            point.y = 320 - ((float(pos['y']) - 0.03) * 600) / 0.9
            point.x = 300 - ((float(pos['x']) - 0.065) * 420) / 0.865
            point.z = 100.0
            valid, singular, msg = is_within_workspace(point.x, point.y, point.z)
            if valid:
                self.hand_pub.publish(point)
                if singular:
                    warn_msg = f"⚠️ Singularity: {msg} | ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
                    self.get_logger().warn(warn_msg)
                    self.error_pub.publish(String(data=warn_msg))
                else:
                    self.get_logger().info(f"✅ Published /hand_position: {point}")
                # WebSocket으로 변환된 좌표 전송
                self.send_to_ws_clients(point)
            else:
                error_msg = f"❌ {msg} | ({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
                self.get_logger().error(error_msg)
                self.error_pub.publish(String(data=error_msg))
        else:
            msg = String()
            msg.data = message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published /gesture_cmd: {msg.data}")

    def send_to_ws_clients(self, point):
        if self.ws_clients and self.websocket_loop:
            msg = json.dumps({'x': point.x, 'y': point.y, 'z': point.z})
            for client in self.ws_clients.copy():
                asyncio.run_coroutine_threadsafe(client.send(msg), self.websocket_loop)
        else:
            print("[WS] No clients or websocket_loop not set.")

    async def ws_handler(self, websocket):
        self.ws_clients.add(websocket)
        try:
            while True:
                msg = await websocket.recv()
                print(f"📥 WS Message: {msg}")
                self.handle_incoming_message(msg)
                await websocket.send("ACK")
        except websockets.exceptions.ConnectionClosed:
            print("❌ WebSocket client disconnected.")
        finally:
            self.ws_clients.discard(websocket)

    def start_websocket_server(self):
        async def run():
            async with websockets.serve(self.ws_handler, '192.168.110.121', 20000):
                print("WebSocket server started on ws://192.168.110.121:20000")
                await asyncio.Future()

        loop = asyncio.new_event_loop()
        self.websocket_loop = loop         # 중요: 루프 저장
        asyncio.set_event_loop(loop)
        loop.run_until_complete(run())

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