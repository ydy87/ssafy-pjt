import socket
import gpiod
import time
import threading
import queue

# GPIO 핀 설정
SERVO_PIN = 18
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22

# 서보 각도
CENTER_ANGLE = 135
RIGHT_ANGLE = 175
LEFT_ANGLE = 95

# 통신 설정
HOST = '0.0.0.0'
PORT = 8765
SPEED = 0.0002  # 컨베이어 고정 속도
SERVO_TIMEOUT = 1.5  # 명령 없을 때 CENTER 복귀 대기 시간 (초)

class SorterAndConveyor:
    def __init__(self):
        chip = gpiod.Chip('gpiochip0')

        # 서보 설정
        self.servo_line = chip.get_line(SERVO_PIN)
        self.servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

        # 컨베이어 설정
        self.dir_line = chip.get_line(DIR_PIN)
        self.step_line = chip.get_line(STEP_PIN)
        self.enable_line = chip.get_line(ENABLE_PIN)

        self.dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
        self.step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
        self.enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)

        # 상태 변수
        self.motor_running = False
        self.motor_thread = None

        self.servo_queue = queue.Queue()
        self.current_servo_cmd = None
        self.last_servo_time = time.time()

        threading.Thread(target=self.servo_worker, daemon=True).start()
        threading.Thread(target=self.servo_watchdog_worker, daemon=True).start()

        print("[INFO] Sorter + Conveyor system initialized.")

    # 서보 제어
    def set_servo(self, angle):
        pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
        for _ in range(10):
            self.servo_line.set_value(1)
            time.sleep(pulse_width)
            self.servo_line.set_value(0)
            time.sleep(0.02 - pulse_width)

    def servo_worker(self):
        while True:
            cmd = self.servo_queue.get()
            self.current_servo_cmd = cmd

            if cmd == "back_panel":
                print("[ACTION] Moving servo LEFT")
                self.set_servo(LEFT_ANGLE)
            elif cmd == "board_panel":
                print("[ACTION] Moving servo RIGHT")
                self.set_servo(RIGHT_ANGLE)
            elif cmd == "center":
                print("[ACTION] Returning servo to CENTER")
                self.set_servo(CENTER_ANGLE)
            else:
                print(f"[WARN] Unknown servo command: {cmd}")

    def servo_watchdog_worker(self):
        while True:
            time.sleep(0.1)
            if self.current_servo_cmd in ['back_panel', 'board_panel']:
                elapsed = time.time() - self.last_servo_time
                if elapsed > SERVO_TIMEOUT:
                    print("[INFO] No recent command - moving servo to CENTER")
                    self.current_servo_cmd = "center"
                    self.servo_queue.put("center")

    # 컨베이어 제어
    def start_motor(self):
        if self.motor_running:
            return
        self.dir_line.set_value(0)
        self.enable_line.set_value(0)
        self.motor_running = True
        self.motor_thread = threading.Thread(target=self.run_motor)
        self.motor_thread.start()
        print("[ACTION] Conveyor started")

    def stop_motor(self):
        if self.motor_running:
            self.motor_running = False
            if self.motor_thread:
                self.motor_thread.join()
            self.enable_line.set_value(1)
            print("[ACTION] Conveyor stopped")

    def run_motor(self):
        while self.motor_running:
            self.step_line.set_value(1)
            time.sleep(SPEED)
            self.step_line.set_value(0)
            time.sleep(SPEED)

    # 메시지 처리
    def handle_message(self, msg):
        if msg == '9':
            self.start_motor()
        elif msg == '10':
            self.stop_motor()
        elif msg in ['back_panel', 'board_panel']:
            self.last_servo_time = time.time()
            if msg != self.current_servo_cmd:
                with self.servo_queue.mutex:
                    self.servo_queue.queue.clear()
                self.servo_queue.put(msg)
        else:
            print(f"[WARN] Unknown message: {msg}")

    def cleanup(self):
        self.stop_motor()
        self.dir_line.release()
        self.step_line.release()
        self.enable_line.release()
        self.servo_line.release()
        print("[INFO] GPIO released and cleanup complete.")

# --- 멀티 클라이언트 처리 ---

def client_handler(conn, addr, controller):
    print(f"[INFO] Connected by {addr}")
    with conn:
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    print(f"[INFO] {addr} disconnected.")
                    break
                msg = data.decode().strip()
                print(f"[CMD] Received from {addr}: {msg}")
                controller.handle_message(msg)
            except Exception as e:
                print(f"[ERROR] Connection with {addr} lost: {e}")
                break

def start_server(controller):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        print(f"[INFO] Server listening on {HOST}:{PORT}")

        while True:
            conn, addr = s.accept()
            threading.Thread(target=client_handler, args=(conn, addr, controller), daemon=True).start()

# --- 실행 ---

if __name__ == '__main__':
    controller = SorterAndConveyor()
    try:
        start_server(controller)
    except KeyboardInterrupt:
        print("[INFO] Keyboard interrupt - exiting")
    finally:
        controller.cleanup()
