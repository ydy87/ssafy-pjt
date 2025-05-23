import socket
import gpiod
import time
import threading

# GPIO 핀 번호
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22

HOST = '0.0.0.0'
PORT = 8765
SPEED = 0.0002  # 고정 속도

class ConveyorController:
    def __init__(self):
        chip = gpiod.Chip('gpiochip0')
        self.dir_line = chip.get_line(DIR_PIN)
        self.step_line = chip.get_line(STEP_PIN)
        self.enable_line = chip.get_line(ENABLE_PIN)

        self.dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
        self.step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
        self.enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)

        self.motor_running = False
        self.thread = None

        print("[INFO] Conveyor controller ready.")

    def start_motor(self):
        if self.motor_running:
            return  # 이미 동작 중이면 무시

        self.dir_line.set_value(0)  # 정방향
        self.enable_line.set_value(0)
        self.motor_running = True

        self.thread = threading.Thread(target=self.run_motor)
        self.thread.start()
        print("[ACTION] Motor started (simple mode)")

    def stop_motor(self):
        if self.motor_running:
            self.motor_running = False
            if self.thread:
                self.thread.join()
            self.enable_line.set_value(1)
            print("[ACTION] Motor stopped")

    def run_motor(self):
        while self.motor_running:
            self.step_line.set_value(1)
            time.sleep(SPEED)
            self.step_line.set_value(0)
            time.sleep(SPEED)

    def handle_message(self, message):
        if message == '9':
            self.start_motor()
        elif message == '10':
            self.stop_motor()
        else:
            print(f"[WARN] Unknown message: {message}")

    def cleanup(self):
        self.stop_motor()
        self.dir_line.release()
        self.step_line.release()
        self.enable_line.release()

def start_server(controller):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[INFO] Listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        print(f"[INFO] Connected by {addr}")

        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    print("[INFO] Client disconnected.")
                    break
                message = data.decode().strip()
                controller.handle_message(message)

if __name__ == '__main__':
    controller = ConveyorController()
    try:
        start_server(controller)
    except KeyboardInterrupt:
        print("[INFO] Keyboard interrupt - exiting")
    finally:
        controller.cleanup()
