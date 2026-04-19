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

try:
    while True:
        action = leader.get_action()
        follower.send_action(action)
        time.sleep(1/freq)
except KeyboardInterrupt:
    print("\nStopping teleop...")
    leader.disconnect()
    follower.disconnect()
