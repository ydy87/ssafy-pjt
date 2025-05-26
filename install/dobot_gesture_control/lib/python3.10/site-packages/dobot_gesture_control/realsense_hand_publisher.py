import cv2
import mediapipe as mp
import socket
import json
import time
import sys
from PyQt5.QtWidgets import QApplication, QMessageBox

HOST = '127.0.0.1'
PORT = 8765
ROI = (0, 0, 960, 720)

# PyQt 앱 초기화
app = QApplication(sys.argv)

def is_palm_open(hand_landmarks):
    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]
    return all(hand_landmarks.landmark[t].y < hand_landmarks.landmark[p].y for t, p in zip(tips, pips))

def is_fist(hand_landmarks):
    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]
    extended = sum(hand_landmarks.landmark[t].y < hand_landmarks.landmark[p].y for t, p in zip(tips, pips))
    return extended <= 1

def classify_gesture(hand_landmarks, handed):
    index_tip = hand_landmarks.landmark[8]
    thumb_tip = hand_landmarks.landmark[4]
    wrist = hand_landmarks.landmark[0]

    dx_index = index_tip.x - wrist.x
    dy_index = index_tip.y - wrist.y
    dy_thumb = thumb_tip.y - wrist.y

    if abs(dx_index) > abs(dy_index):
        if dx_index > 0.15:
            return 3
        elif dx_index < -0.15:
            return 4
    else:
        if dy_index < -0.15:
            return 2
        elif dy_index > 0.15:
            return 1

    if handed == 'Right':
        if dy_thumb < -0.15:
            return 6
        elif dy_thumb > 0.15:
            return 5
    else:
        if dy_thumb > 0.15:
            return 6
        elif dy_thumb < -0.15:
            return 5

    return 0

class HandGesturePublisher:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.last_send_time = 0
        self.send_interval = 0.3
        self.overlay = cv2.imread("coordinate.png")
        self.overlay = cv2.resize(self.overlay, (ROI[2] - ROI[0], ROI[3] - ROI[1]))

        self.left_gesture_start_time = None
        self.last_left_command = None

    def run(self):
        cap = cv2.VideoCapture(0)
        cv2.namedWindow("Hand Gesture Controller", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Hand Gesture Controller", 960, 720)  
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)
            h, w, _ = frame.shape

            x1, y1, x2, y2 = ROI
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            roi = frame[y1:y2, x1:x2]
            overlay_resized = cv2.resize(self.overlay, (roi.shape[1], roi.shape[0]))
            blended = cv2.addWeighted(roi, 0.5, overlay_resized, 0.5, 0)
            frame[y1:y2, x1:x2] = blended


            if results.multi_hand_landmarks and results.multi_handedness:
                for i, hand in enumerate(results.multi_hand_landmarks):
                    handed = results.multi_handedness[i].classification[0].label
                    index_tip = hand.landmark[8]
                    cx, cy = int(index_tip.x * w), int(index_tip.y * h)

                    gesture_code = classify_gesture(hand, handed)
                    skip_position = False

                    if handed == 'Right':
                        if gesture_code == 5:
                            self.sock.sendall(b'1')
                            print("Z Down (1)")
                            skip_position = True
                        elif gesture_code == 6:
                            self.sock.sendall(b'2')
                            print("Z Up (2)")
                            skip_position = True

                        if not skip_position and x1 <= cx <= x2 and y1 <= cy <= y2:
                            now = time.time()
                            if now - self.last_send_time >= self.send_interval:
                                norm_x = index_tip.x
                                norm_y = index_tip.y
                                xy = {"x": round(norm_x, 3), "y": round(norm_y, 3)}
                                self.sock.sendall((json.dumps(xy) + '\n').encode('utf-8'))
                                print("XY Sent:", xy)
                                self.last_send_time = now

                    elif handed == 'Left':
                        now = time.time()
                        if gesture_code in (5, 6):  # Thumb down or up
                            command = '4' if gesture_code == 5 else '3'
                            label = "석션 ON" if command == '4' else "석션 OFF"

                            if self.left_gesture_start_time is None:
                                self.left_gesture_start_time = now
                                self.last_left_command = None
                            elif now - self.left_gesture_start_time >= 0.3:
                                if self.last_left_command != command:
                                    reply = QMessageBox.question(
                                        None,
                                        '석션 제어',
                                        f"{label} 하시겠습니까?",
                                        QMessageBox.Ok | QMessageBox.Cancel,
                                        QMessageBox.Cancel
                                    )
                                    if reply == QMessageBox.Ok:
                                        self.sock.sendall(command.encode('utf-8'))
                                        print(f"{label} ({command})")
                                        self.last_left_command = command
                        else:
                            self.left_gesture_start_time = None
                            self.last_left_command = None

                        if is_fist(hand):
                            self.sock.sendall(b'5')
                            print("Conveyor Stop (5)")
                        elif is_palm_open(hand):
                            self.sock.sendall(b'6')
                            print("Conveyor Resume (6)")

                    if handed == 'Right' and not skip_position:
                        # 검지 끝 좌표 시각화
                        ix, iy = int(index_tip.x * w), int(index_tip.y * h)
                        cv2.circle(frame, (ix, iy), 8, (0, 255, 0), -1)  # 초록색 원
                    else:
                        # 왼손 또는 skip 상황이면 전체 마커 표시
                        mp.solutions.drawing_utils.draw_landmarks(
                            frame, hand, self.mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Hand Gesture Controller", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cap.release()
        self.sock.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    HandGesturePublisher().run()
