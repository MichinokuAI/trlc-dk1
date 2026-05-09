"""Host script: run next to the physical DK1 leader and broadcast its actions
over ZMQ so a remote :class:`DK1LeaderRemote` client can drive a follower."""

import argparse
import json
import logging
import time

import zmq

from lerobot_robot_trlc_dk1.leader import DK1Leader, DK1LeaderConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port for the leader (e.g. /dev/cu.usbmodemXXXX)")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument("--bind", default="tcp://*:5557", help="ZMQ bind address for action PUSH socket")
    parser.add_argument("--freq", type=float, default=200.0, help="Loop frequency [Hz]")
    args = parser.parse_args()

    leader = DK1Leader(DK1LeaderConfig(port=args.port, baudrate=args.baudrate))
    leader.connect()
    logger.info("Leader connected.")

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.bind(args.bind)
    logger.info("Broadcasting actions on %s", args.bind)

    period = 1.0 / args.freq
    try:
        while True:
            loop_start = time.perf_counter()
            action = leader.get_action()
            try:
                sock.send_string(json.dumps(action), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(period - elapsed, 0.0))
    except KeyboardInterrupt:
        logger.info("Stopping leader host...")
    finally:
        sock.close()
        ctx.term()
        leader.disconnect()


if __name__ == "__main__":
    main()
