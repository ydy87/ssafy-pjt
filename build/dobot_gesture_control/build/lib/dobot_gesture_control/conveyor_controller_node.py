import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import time
import threading

# GPIO 핀 번호
DIR_PIN = 17
STEP_PIN = 27
ENABLE_PIN = 22

class ConveyorController(Node):
    def __init__(self):
        super().__init__('conveyor_controller_node')

        # GPIO 설정
        chip = gpiod.Chip('gpiochip0')
        self.dir_line = chip.get_line(DIR_PIN)
        self.step_line = chip.get_line(STEP_PIN)
        self.enable_line = chip.get_line(ENABLE_PIN)

        self.dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
        self.step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
        self.enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)

        # /gesture_cmd 구독
        self.subscription = self.create_subscription(
            String,
            '/gesture_cmd',
            self.gesture_callback,
            10
        )

        # 상태 변수
        self.motor_running = False
        self.motor_direction = 0
        self.Speed = 0.0002
        self.thread = None
        self.InitialSpeed = 0.0005
        self.TargetSpeed = 0.0001
        self.RATIO = 0.0000005
        self.graceful_stop = False



        self.get_logger().info('Conveyor controller node started.')

    def gesture_callback(self, msg):
        code = msg.data.strip()
        if code == '9':
            self.get_logger().info("Gesture 9: Start conveyor (CW)")
            self.start_motor(direction=1)
        elif code == '10':
            self.get_logger().info("Gesture 10: Stop conveyor")
            self.stop_motor()

    def start_motor(self, direction):
        if self.motor_running:
            self.stop_motor()
            time.sleep(0.1)

        self.motor_direction = direction
        self.dir_line.set_value(0 if direction == 1 else 1)
        self.enable_line.set_value(0)
        self.motor_running = True
        self.thread = threading.Thread(target=self.step_motor)
        self.thread.start()
        self.get_logger().info(f'Motor started: {"CW" if direction == 1 else "CCW"}')

    def stop_motor(self):
        if self.motor_running:
            self.get_logger().info("Initiating graceful stop")
            self.graceful_stop = True
            if self.thread is not None:
                self.thread.join()  # 감속 종료될 때까지 기다림


    def step_motor(self):
        speed = self.InitialSpeed
        accelerating = True

        while self.motor_running or self.graceful_stop:
            if self.graceful_stop:
                speed += self.RATIO
                if speed >= self.InitialSpeed:
                    speed = self.InitialSpeed
                    break
            elif self.motor_direction != 0:
                if accelerating and speed > self.TargetSpeed:
                    speed -= self.RATIO
                else:
                    speed = self.TargetSpeed
                    accelerating = False

            self.step_line.set_value(1)
            time.sleep(speed)
            self.step_line.set_value(0)
            time.sleep(speed)

        self.motor_running = False
        self.graceful_stop = False
        self.enable_line.set_value(1)
        self.get_logger().info("Motor completely stopped (with deceleration)")


    def destroy_node(self):
        self.stop_motor()
        time.sleep(0.1)
        self.dir_line.release()
        self.step_line.release()
        self.enable_line.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
