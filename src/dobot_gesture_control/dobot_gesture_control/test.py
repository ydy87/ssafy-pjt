import cv2
import mediapipe as mp
import socket
import time
import sys
from PyQt5.QtWidgets import QApplication, QMessageBox

# PyQt 앱 초기화 (전역에서 1회만)
app = QApplication(sys.argv)

# Mediapipe hands 초기화
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1,
                       min_detection_confidence=0.7, min_tracking_confidence=0.5)

# 웹캠 열기
cap = cv2.VideoCapture(0)

# 소켓 설정 (수신 쪽 서버 주소와 포트로 변경하세요)
HOST = '192.168.110.146'   # ROS 서버 IP 주소
PORT = 8765              # 포트 번호

# 소켓 설정 Test
# HOST = '127.0.0.1'
# PORT = 20000

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

# Gripper 상태 관리
gripper_on = False
palm_start_time = None
last_send_time = 0
send_interval = 0.3  # 최소 0.3초 간격으로 명령 전송

# 손바닥 펼침 판단 함수
def is_palm_open(hand_landmarks):
    # 각 손가락의 tip이 pip보다 높이 있으면 펼친 상태 (y값 작을수록 위쪽)
    fingers_extended = 0
    finger_tips = [8, 12, 16, 20]   # 검지, 중지, 약지, 새끼
    finger_pips = [6, 10, 14, 18]   # 해당 손가락의 PIP 관절

    for tip_idx, pip_idx in zip(finger_tips, finger_pips):
        tip_y = hand_landmarks.landmark[tip_idx].y
        pip_y = hand_landmarks.landmark[pip_idx].y
        if tip_y < pip_y:
            fingers_extended += 1

    # 4개 손가락 모두 펴진 경우만 손바닥으로 간주
    return fingers_extended == 4

# 오른손 주먹 판단 (0~1개만 펴진 경우)
def is_right_fist(hand_landmarks):
    fingers_extended = 0
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]

    for tip_idx, pip_idx in zip(finger_tips, finger_pips):
        if hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y:
            fingers_extended += 1
    return fingers_extended <= 1

# 오른손 펼침 판단
def is_right_open(hand_landmarks):
    fingers_extended = 0
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]

    for tip_idx, pip_idx in zip(finger_tips, finger_pips):
        if hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y:
            fingers_extended += 1
    return fingers_extended == 4

# 제스처 분류 함수
def classify_gesture(hand_landmarks):
    # 각 landmark 좌표 참조 (Tip: index=8, thumb=4, wrist=0)
    index_tip = hand_landmarks.landmark[8]
    thumb_tip = hand_landmarks.landmark[4]
    wrist = hand_landmarks.landmark[0]

    dx_index = index_tip.x - wrist.x
    dy_index = index_tip.y - wrist.y

    dx_thumb = thumb_tip.x - wrist.x
    dy_thumb = thumb_tip.y - wrist.y

    # 검지 방향 기준
    if abs(dx_index) > abs(dy_index):
        if dx_index > 0.15:
            return 3  # 오른쪽
        elif dx_index < -0.15:
            return 4  # 왼쪽
    else:
        if dy_index < -0.15:
            return 2  # 위
        elif dy_index > 0.15:
            return 1  # 아래

    # 엄지 방향 (보조 분류)
    if dy_thumb < -0.15:
        return 6  # 엄지 위
    elif dy_thumb > 0.15:
        return 5  # 엄지 아래

    return 0  # 인식 실패

# PyQt 메시지박스 호출
def show_confirmation(gripper_state):
    msg = "Gripper OFF 하시겠습니까?" if gripper_state else "Gripper ON 하시겠습니까?"
    reply = QMessageBox.question(None, 'Gripper 제어', msg,
                                 QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Cancel)
    return reply == QMessageBox.Ok


try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.flip(frame, 1)  # 좌우 반전 추가
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)

        current_command = 0
        palm_open_now = False

        if results.multi_hand_landmarks and results.multi_handedness:
            for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                handedness = results.multi_handedness[i].classification[0].label
                

                mp.solutions.drawing_utils.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                if handedness == 'Left':
                    current_command = classify_gesture(hand_landmarks)

                    # 손바닥 상태 감지
                    if is_palm_open(hand_landmarks):
                        palm_open_now = True
                        if palm_start_time is None:
                            palm_start_time = time.time()
                        elif time.time() - palm_start_time >= 1.0:
                            # 팝업 표시
                            if show_confirmation(gripper_on):
                                command = 8 if gripper_on else 7
                                client_socket.sendall(str(command).encode('utf-8'))
                                gripper_on = not gripper_on
                            palm_start_time = None
                            break  # 팝업 중단 후 프레임 넘김
                    else:
                        palm_start_time = None
                        current_command = classify_gesture(hand_landmarks)
                        # ✅ 전송 간격 제한 적용
                        if current_command != 0 and time.time() - last_send_time >= send_interval:
                            client_socket.sendall(str(current_command).encode('utf-8'))
                            last_send_time = time.time()

                elif handedness == 'Right':
                    if is_right_fist(hand_landmarks):
                        client_socket.sendall(b'9')
                        current_command = 9
                    elif is_right_open(hand_landmarks):
                        client_socket.sendall(b'10')
                        current_command = 10

        if not palm_open_now:
            palm_start_time = None

        # 화면 출력
        cv2.putText(frame, f"Command: {current_command}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Hand Gesture for Dobot control", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    client_socket.close()