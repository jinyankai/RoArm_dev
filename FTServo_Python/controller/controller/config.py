class Config:

    movement_sensitivity: float = 0.002
    mouth_open_threshold: float = 0.02

    max_frame_skip: int = 2
    idle_timeout: float = 1.5
    reset_hold_time: float = 3.0

    # Joint angle limits
    min_base_angle: float = -3.14
    max_base_angle: float = 3.14
    min_vertical_angle: float = -1.57
    max_vertical_angle: float = 1.57

    port: str = "COM8"
    baudrate: int = 115200

