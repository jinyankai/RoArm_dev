import json
import threading
from collections import deque
import serial
import time
from .config import Config

class RobotController:
    """Handles sending commands to the robot arm"""

    def __init__(self, config: Config):
        self.config = config
        self.current_gripper_state: int = 0
        self.working_flag = 0
        self.command = deque(maxlen = 10)
        self.ser = None
        self.command_lock = threading.Lock()
        self.stop_event = None

    def start_thread(self):
        self.working_flag = 1
        if self.working_flag:
            self.thread = threading.Thread(target=self.send_command_async)
            self.thread.daemon = True
            self.thread.start()

    def update_command(self,command):
        with self.command_lock:
            self.command.append(command)

    def send_command_init(self) -> None:
        ser = None
        serial_recv_thread = None
        self.stop_event = threading.Event()

        try:
            # 创建串口连接
            ser = serial.Serial(self.config.port, baudrate=self.config.baudrate, timeout=0.1)
            self.ser = ser

            if not self.ser.is_open:
                self.ser.open()

            def read_serial():
                while not self.stop_event.is_set():
                    try:
                        if ser.in_waiting > 0:
                            data = ser.readline().decode('utf-8').strip()
                            if data:
                                print(f"[ARM] Received: {data}")
                    except Exception as e:
                        if not self.stop_event.is_set():  # 仅在未停止时打印错误
                            print(f"[ARM] Serial read error: {e}")

        except Exception as e:
            print(f"init serial wrong: {e}")


    def send_command_async(self):
        # 发送控制指令
        try:
            arm_command = self.command.pop()
            json_data = json.dumps(arm_command)
            self.ser.write(json_data.encode() + b'\n')
            print(f"[ARM] Command sent: {arm_command}")
            # 等待指令执行完成
            time.sleep(2)
        except Exception as e:
            print(f"[ARM] Control error: {e}")
        finally:
            # 先通知线程停止
            self.stop_event.set()

            # 等待线程安全退出
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=0.5)

            # 然后关闭串口
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    print("[ARM] Serial port closed")
                except Exception as e:
                    print(f"[ARM] Error closing port: {e}")

