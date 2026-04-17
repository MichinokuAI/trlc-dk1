from lerobot_robot_trlc_dk1.follower import DK1Follower, DK1FollowerConfig
from lerobot_robot_trlc_dk1.leader import DK1Leader, DK1LeaderConfig
import time


follower_config = DK1FollowerConfig(
    port="/dev/tty.usbmodem208634AD47431",
    joint_velocity_scaling=0.2,
)

leader_config = DK1LeaderConfig(
    port="/dev/cu.usbmodem5A680107081"
)
leader = DK1Leader(leader_config)
leader.bus.port_handler.baudrate = 57600
leader.connect()

follower = DK1Follower(follower_config)
follower.connect()

freq = 200 # Hz
print_hz = 10
print_interval = max(1, freq // print_hz)
loop_count = 0
inverted_joints = ("joint_2.pos", "joint_3.pos", "joint_4.pos")

try:
    while True:
        action = leader.get_action()
        # Reverse control direction for selected joints so leader motion maps correctly to follower.
        for joint in inverted_joints:
            action[joint] = -action[joint]
        follower.send_action(action)

        for motor in follower.motors.values():
            follower.control.refresh_motor_status(motor)

        if loop_count % print_interval == 0:
            follower_pos = {key: float(motor.getPosition()) for key, motor in follower.motors.items()}
            print(f"Follower position: {follower_pos}")

        loop_count += 1
        time.sleep(1/freq)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
