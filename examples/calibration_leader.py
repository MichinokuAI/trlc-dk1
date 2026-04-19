from __future__ import annotations

import argparse

from lerobot_robot_trlc_dk1.leader import DK1Leader, DK1LeaderConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive leader calibration: set Homing_Offset after manual resting pose."
    )
    parser.add_argument(
        "--port",
        default="/dev/cu.usbmodem5A680107081",
        help="Leader serial port (Dynamixel).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Leader baudrate.",
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        default=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        help="Joint names to calibrate.",
    )
    parser.add_argument(
        "--target-position",
        type=int,
        default=2048,
        help="Target raw Present_Position for the resting pose.",
    )
    parser.add_argument(
        "--skip-reset-offsets",
        action="store_true",
        help="Skip resetting all Homing_Offset to 0 before calibration.",
    )
    parser.add_argument(
        "--keep-gripper-torque",
        action="store_true",
        help="Do not disable gripper torque during calibration.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    leader = DK1Leader(DK1LeaderConfig(port=args.port))
    leader.bus.port_handler.baudrate = args.baudrate
    leader.connect()

    try:
        print(f"Connected leader on {args.port} @ {args.baudrate} bps")

        if not args.keep_gripper_torque:
            leader.bus.write("Torque_Enable", "gripper", 0, normalize=False)
            print("Gripper torque disabled.")

        if not args.skip_reset_offsets:
            leader.bus.sync_write("Homing_Offset", 0, normalize=False)
            print("Reset all Homing_Offset to 0.")

        input("Place leader in resting position, then press Enter...")

        print("Calibrating joints:")
        for joint in args.joints:
            position = int(leader.bus.read("Present_Position", joint, normalize=False))
            offset = int(args.target_position - position)
            leader.bus.write("Homing_Offset", joint, offset, normalize=False)
            print(f"  {joint}: position={position}, offset={offset}")

        print("Current Homing_Offset:")
        print(leader.bus.sync_read("Homing_Offset", normalize=False))

        print("Current Present_Position:")
        print(leader.bus.sync_read(normalize=False, data_name="Present_Position"))

    finally:
        if leader.is_connected:
            leader.disconnect()
        print("Disconnected leader.")


if __name__ == "__main__":
    main()
