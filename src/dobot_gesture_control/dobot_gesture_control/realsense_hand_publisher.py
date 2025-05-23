import cv2
import mediapipe as mp
import socket
import json
import time

ROI = (0, 0, 640, 480)  # 너비 320px, 높이 240px

# TCP 서버 주소
HOST = '127.0.0.1'  # gesture_bridge_node 실행중인 IP
PORT = 8765

class HandTracker:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")

        # Overlay 이미지 불러오기 및 ROI 크기로 리사이즈
        self.overlay = cv2.imread("coordinate.png")
        roi_width = ROI[2] - ROI[0]
        roi_height = ROI[3] - ROI[1]
        self.overlay = cv2.resize(self.overlay, (roi_width, roi_height))

    def run(self):
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            frame = cv2.flip(frame, 1)
            if not ret:
                break

            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(image_rgb)

            h, w, _ = frame.shape
            roi_x1, roi_y1, roi_x2, roi_y2 = ROI

            # ROI 사각형 그리기
            cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

            # ROI 위에 overlay 이미지 반투명하게 덧씌우기
            roi_area = frame[roi_y1:roi_y2, roi_x1:roi_x2]
            blended = cv2.addWeighted(roi_area, 0.5, self.overlay, 0.5, 0)
            frame[roi_y1:roi_y2, roi_x1:roi_x2] = blended

            if results.multi_hand_landmarks:
                hand = results.multi_hand_landmarks[0]
                # 검지 끝 (INDEX_FINGER_TIP) 사용
                fingertip = hand.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                cx = int(fingertip.x * w)
                cy = int(fingertip.y * h)

                if roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2:
                    norm_x = fingertip.x
                    norm_y = fingertip.y

                    payload = {
                        "x": round(norm_x, 3),
                        "y": round(norm_y, 3)
                    }

                    try:
                        self.sock.sendall((json.dumps(payload) + '\n').encode('utf-8'))
                        print(f"Sent: {payload}")
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    except Exception as e:
                        print(f"Socket send error: {e}")
                        break

            cv2.imshow('hand tracker', frame)
            if cv2.waitKey(1) == 27:
                break

        cap.release()
        self.sock.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    tracker = HandTracker()
    tracker.run()
