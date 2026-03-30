from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from lerobot_robot_trlc_dk1.motors.DM_Control_Python.DM_CAN import *

import time


follower_config = DK1FollowerConfig(
    port="/dev/ttyACM1",
)
follower = DK1Follower(follower_config)

follower.control = MotorControl(
    channel=follower_config.port,
    interface=follower_config.can_interface,
    bitrate=follower_config.can_bitrate,
    tty_baudrate=follower_config.can_tty_baudrate,
    receive_timeout=follower_config.can_receive_timeout,
)

for key, motor in follower.motors.items():
    follower.control.addMotor(motor)
    for _ in range(3):
        follower.control.refresh_motor_status(motor)
        time.sleep(0.01)
    
    if follower.control.read_motor_param(motor, DM_variable.CTRL_MODE) is not None:
        print(f"{key} ({motor.MotorType.name}) is connected.")
    else:
        raise Exception(f"Unable to read from {key} ({motor.MotorType.name}).")

for key, motor in follower.motors.items():
    follower.control.set_zero_position(motor)
    print(f"{key} ({motor.MotorType.name}) set to zero position.")
    
follower.control.close()
