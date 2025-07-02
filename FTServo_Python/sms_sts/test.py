import sys, time, json, threading
from scservo_sdk import PortHandler, scscl
import serial

# ---------- 舵机初始化 ----------
SCS_PORT = 'COM8'
SCS_BAUD = 1_000_000
SCS_IDS  = range(1, 12)
TARGET_POS = [520, 775, 500, 880, 600, 880, 490, 850, 630, 860, 700]
SPEED = 100

port = PortHandler(SCS_PORT)
packet = scscl(port)

assert port.openPort(),  "打开舵机串口失败"
assert port.setBaudRate(SCS_BAUD), "修改波特率失败"

for i, pos in zip(SCS_IDS, TARGET_POS):
    res, err = packet.RegWritePos(i, pos, 0, SPEED)
    if res == 0:  # COMM_SUCCESS == 0
        print(f"ID {i} 指令发送 OK")
    else:
        print(f"ID {i} 通信错误: {packet.getTxRxResult(res)} / {packet.getRxPacketError(err)}")

packet.RegAction()
time.sleep(0.5)        # 给舵机一点时间

port.closePort()       # 到此舵机动作已下发，可放心关口

# ---------- MCU JSON 协议 ----------
MCU_PORT  = 'COM9'      # *** 若 MCU 与舵机共线就还是 COM8，否则改实际端口
MCU_BAUD  = 115200

ser = serial.Serial(MCU_PORT, MCU_BAUD, timeout=0.2)

def read_serial():
    while ser.is_open:
        try:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                print(f"[MCU] {line.strip()}")
        except Exception as e:
            print("串口读取异常:", e)

threading.Thread(target=read_serial, daemon=True).start()

# 休息位指令
cmd = {"T": 102, "base": 3.14, "shoulder": 6.28, "elbow": 1.57,
       "hand": 3.14, "spd": 0, "acc": 10}
ser.write((json.dumps(cmd) + '\n').encode())

# --- 业务循环 / 调试 ---
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("串口已关闭")
