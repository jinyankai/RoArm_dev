# robot_control.py
import serial
import threading
import json
import time
import sys
import os
from scservo_sdk import *


def control_arm(arm_command, com_port="COM11", baudrate=115200):
    """
    控制机械臂

    参数:
    arm_command (dict): 机械臂控制指令
    com_port (str): 串口端口, 默认 "COM11"

    baudrate (int): 波特率, 默认 115200
    """
    ser = None
    serial_recv_thread = None
    stop_event = threading.Event()

    try:
        # 创建串口连接
        ser = serial.Serial(com_port, baudrate=baudrate, dsrdtr=None)
        ser.setRTS(False)
        ser.setDTR(False)

        if not ser.is_open:
            ser.open()

        # 串口数据读取线程
        def read_serial():
            while not stop_event.is_set():
                try:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('utf-8').strip()
                        if data:
                            print(f"[ARM] Received: {data}")
                except Exception as e:
                    if not stop_event.is_set():  # 仅在未停止时打印错误
                        print(f"[ARM] Serial read error: {e}")
                    break

        # 启动读取线程
        serial_recv_thread = threading.Thread(target=read_serial, daemon=True)
        serial_recv_thread.start()

        # 发送控制指令
        json_data = json.dumps(arm_command)
        ser.write(json_data.encode() + b'\n')
        print(f"[ARM] Command sent: {arm_command}")

        # 等待指令执行完成
        time.sleep(2)

    except Exception as e:
        print(f"[ARM] Control error: {e}")
    finally:
        # 先通知线程停止
        stop_event.set()

        # 等待线程安全退出
        if serial_recv_thread and serial_recv_thread.is_alive():
            serial_recv_thread.join(timeout=0.5)

        # 然后关闭串口
        if ser and ser.is_open:
            try:
                ser.close()
                print("[ARM] Serial port closed")
            except Exception as e:
                print(f"[ARM] Error closing port: {e}")


def control_hand(hand_positions, speed=100, com_port="COM12", baudrate=1000000):
    """
    控制机械手(舵机)

    参数:
    hand_positions (list): 11个舵机的位置值列表
    speed (int): 舵机运动速度, 默认 100
    com_port (str): 串口端口, 默认 "COM8"
    baudrate (int): 波特率, 默认 1000000
    """
    try:
        # 创建舵机控制器
        portHandler = PortHandler(com_port)
        packetHandler = scscl(portHandler)

        # 打开端口
        if not portHandler.openPort():
            print("[HAND] Failed to open port")
            return

        # 设置波特率
        if not portHandler.setBaudRate(baudrate):
            print("[HAND] Failed to set baudrate")
            return

        # 发送位置指令到所有舵机
        for scs_id, position in enumerate(hand_positions, start=1):
            comm_result, error = packetHandler.RegWritePos(scs_id, position, 0, speed)

            if comm_result != COMM_SUCCESS:
                print(f"[HAND] Servo {scs_id} write error: {packetHandler.getTxRxResult(comm_result)}")
            if error != 0:
                print(f"[HAND] Servo {scs_id} error: {packetHandler.getRxPacketError(error)}")

        # 执行动作
        packetHandler.RegAction()
        print(f"[HAND] Command sent to {len(hand_positions)} servos")

        # 等待指令执行完成
        time.sleep(1)

    except Exception as e:
        print(f"[HAND] Control error: {e}")
    finally:
        if 'portHandler' in locals() and portHandler.is_open:
            portHandler.closePort()
            print("[HAND] Port closed")


# 示例使用
if __name__ == "__main__":
    # 机械臂控制示例
    arm_command = {
        "T": 102,
        "base": 0,
        "shoulder": 1.57,
        "elbow": 1.57,
        "hand": 3.14,
        "spd": 0,
        "acc": 10
    }
    control_arm(arm_command)

    # 机械手控制示例
    hand_positions = [520, 775, 500, 880, 600, 880, 490, 850, 630, 860, 700]
    control_hand(hand_positions, speed=100)