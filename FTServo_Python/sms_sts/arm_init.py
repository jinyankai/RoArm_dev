import serial
import threading
import json

import time

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')


ser = serial.Serial("COM8", baudrate=115200, timeout=0.2)
ser.setRTS(False)
ser.setDTR(False)

if not ser.is_open:
    ser.open()

serial_recv_thread = threading.Thread()
serial_recv_thread.daemon = True

serial_recv_thread.start()

# test
data = {"T": 1041,
        "x": 200,
        "y": 0,
        "z": 0
        }

# 伸长手臂
# data = {"T": 102,
#         "base": -3.14,
#         "shoulder": -1.57,
#         "elbow": 0,
#         "hand": 0,
#         "spd": 0,
#         "acc": 10
#         }

# 反馈信息
# data = {"T": 105}

# 复位
# data = {"T": 100}

# 休息位置
# data = {"T":102,"base":10,"shoulder":0,"elbow":1.57,"hand":3.14,"spd":0,"acc":10
#         }

json_data = json.dumps(data)

ser.write(json_data.encode() + b'\n')

ser.close()