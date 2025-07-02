# main.py
from robot_control import control_arm, control_hand
import time

# 控制机械臂到休息位置
rest_position = {
    "T": 102,
    "base": 0,
    "shoulder": 1.57,
    "elbow": 1.57,
    "hand": 1.57,
    "spd": 0,
    "acc": 10
}
control_arm(rest_position)

# 控制机械手到抓取位置
grab_positions = [300, 600, 400, 700, 500, 800, 400, 700, 500, 800, 600]
control_hand(grab_positions, speed=150)

# 等待动作完成
time.sleep(2)

# 控制机械手到释放位置
release_positions = [500, 800, 600, 900, 700, 900, 500, 800, 700, 900, 800]
control_hand(release_positions)