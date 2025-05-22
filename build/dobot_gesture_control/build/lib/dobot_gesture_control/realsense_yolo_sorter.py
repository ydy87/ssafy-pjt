import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import socket

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolov5_node')

        # YOLO 모델 불러오기 (사용자 정의 모델)
        self.yolo_model_customed = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ssafy/Downloads/best.pt')

        # RealSense 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # ROS 퍼블리셔 및 타이머 설정
        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.roi_position = (116, 87, 545, 256)  # ROI 설정

        # 소켓 연결 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.110.146', 9999))

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        if 60 < h < 120 and s < 40 and 140 < v < 210:
            return 'white'
        elif ((h <= 10 or h >= 160) and s >= 100 and v >= 100):
            return 'red'
        elif 80 < h < 150 and s > 200 and 80 < v < 130:
            return 'blue'
        return 'unknown'

    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4

        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)

        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))

        return average_color

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        roi_x1, roi_y1, roi_x2, roi_y2 = self.roi_position
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)
        roi_image = color_image[roi_y1:roi_y2, roi_x1:roi_x2]

        results = self.yolo_model_customed(roi_image)
        self.detection_result = String()

        for result in results.xyxy[0]:
            x1, y1, x2, y2, conf, class_id = result[:6]
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            object_roi = roi_image[y1:y2, x1:x2]
            center_color = self.get_center_color(object_roi)
            color_name = self.get_color_name(center_color)
            color_bgr = self.get_color_bgr(color_name)

            label = self.yolo_model_customed.names[int(class_id)]
            self.detection_result.data = f'{label} {color_name}'

            # 라즈베리파이로 클래스 정보 전송
            if label in ['back_panel', 'board_panel']:
                try:
                    self.sock.sendall(label.encode())
                    self.get_logger().info(f'Sent to Pi: {label}')
                except Exception as e:
                    self.get_logger().error(f'Socket send failed: {e}')

            # 화면에 박스와 라벨 표시
            cv2.rectangle(color_image, (roi_x1 + x1, roi_y1 + y1), (roi_x1 + x2, roi_y1 + y2), (0, 255, 0), 2)
            cv2.putText(color_image, f'{label}-{color_name}', (roi_x1 + x1, roi_y1 + y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2)

        cv2.imshow("yolo_roi", color_image)
        cv2.waitKey(1)

        self.detection_publisher.publish(self.detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
