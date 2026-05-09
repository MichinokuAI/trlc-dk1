#   Copyright 2025 The Robot Learning Company UG (haftungsbeschränkt). All rights reserved.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

import json
import logging
import time
from dataclasses import dataclass

from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

logger = logging.getLogger(__name__)


MOTOR_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "gripper",
)


@TeleoperatorConfig.register_subclass("dk1_leader_remote")
@dataclass
class DK1LeaderRemoteConfig(TeleoperatorConfig):
    remote_ip: str
    port_zmq_action: int = 5557
    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5


class DK1LeaderRemote(Teleoperator):
    """
    Remote client for the TRLC-DK1 Leader Arm.

    Mirrors the action interface of :class:`DK1Leader` but receives joint
    positions over ZMQ from a host process running next to the physical leader
    instead of reading them from the local Dynamixel bus. The host is expected
    to publish JSON actions on ``port_zmq_action`` (PUSH).
    """

    config_class = DK1LeaderRemoteConfig
    name = "dk1_leader_remote"

    def __init__(self, config: DK1LeaderRemoteConfig):
        import zmq

        super().__init__(config)
        self._zmq = zmq
        self.config = config

        self.zmq_context = None
        self.zmq_action_socket = None

        self._is_connected = False
        self.last_action: dict[str, float] = {f"{m}.pos": 0.0 for m in MOTOR_NAMES}

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in MOTOR_NAMES}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        pass

    def connect(self, calibrate: bool = False) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        zmq = self._zmq
        self.zmq_context = zmq.Context()

        self.zmq_action_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_action_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_action_socket.connect(
            f"tcp://{self.config.remote_ip}:{self.config.port_zmq_action}"
        )

        poller = zmq.Poller()
        poller.register(self.zmq_action_socket, zmq.POLLIN)
        socks = dict(poller.poll(self.config.connect_timeout_s * 1000))
        if (
            self.zmq_action_socket not in socks
            or socks[self.zmq_action_socket] != zmq.POLLIN
        ):
            raise DeviceNotConnectedError(
                f"Timeout waiting for DK1 leader host at "
                f"{self.config.remote_ip}:{self.config.port_zmq_action}."
            )

        self._is_connected = True
        logger.info(f"{self} connected to {self.config.remote_ip}.")

    def _poll_latest_message(self) -> str | None:
        zmq = self._zmq
        poller = zmq.Poller()
        poller.register(self.zmq_action_socket, zmq.POLLIN)

        try:
            socks = dict(poller.poll(self.config.polling_timeout_ms))
        except zmq.ZMQError as e:
            logger.error(f"ZMQ polling error: {e}")
            return None

        if self.zmq_action_socket not in socks:
            return None

        last_msg = None
        while True:
            try:
                last_msg = self.zmq_action_socket.recv_string(zmq.NOBLOCK)
            except zmq.Again:
                break

        return last_msg

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()

        msg = self._poll_latest_message()
        if msg is not None:
            try:
                payload = json.loads(msg)
            except json.JSONDecodeError as e:
                logger.error(f"Error decoding action JSON: {e}")
                payload = None

            if payload is not None:
                self.last_action = {
                    f"{m}.pos": float(payload.get(f"{m}.pos", self.last_action[f"{m}.pos"]))
                    for m in MOTOR_NAMES
                }

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")

        return dict(self.last_action)

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.zmq_action_socket.close()
        self.zmq_context.term()
        self._is_connected = False

        logger.info(f"{self} disconnected.")
