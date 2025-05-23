import socket
import gpiod
import time
import threading
import queue

SERVO_PIN = 18
CENTER_ANGLE = 135
RIGHT_ANGLE = 175
LEFT_ANGLE = 95

# Setup GPIO
chip = gpiod.Chip('gpiochip0')
servo_line = chip.get_line(SERVO_PIN)
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)

# Servo control
def set_servo(angle):
    pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
    for _ in range(10):
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(0.02 - pulse_width)

# Command queue and worker
cmd_queue = queue.Queue()
last_cmd = None

def servo_worker():
    global last_cmd
    while True:
        cmd = cmd_queue.get()
        if cmd == last_cmd:
            continue  # Skip duplicate
        last_cmd = cmd
        if cmd == "back_panel":
            print("[ACTION] Moving servo LEFT")
            set_servo(LEFT_ANGLE)
        elif cmd == "board_panel":
            print("[ACTION] Moving servo RIGHT")
            set_servo(RIGHT_ANGLE)
        else:
            print("[WARN] Unknown command")
            continue
        time.sleep(2)  # Wait before returning to center
        set_servo(CENTER_ANGLE)

# Start worker thread
threading.Thread(target=servo_worker, daemon=True).start()

# Socket server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("0.0.0.0", 9999))
server.listen(1)
print("[INFO] Raspberry Pi Sorter Server Started on port 9999")

conn, addr = server.accept()
print(f"[INFO] Connected from {addr}")

try:
    while True:
        data = conn.recv(1024).decode().strip()
        if data:
            print(f"[CMD] Received: {data}")
            cmd_queue.put(data)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    servo_line.release()
    conn.close()
    server.close()
