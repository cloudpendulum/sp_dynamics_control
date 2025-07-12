import pyCandle
import time
import json
import math
from dataclasses import dataclass

@dataclass
class UserInfo:
    user_name: str
    max_experiment_length: int
    number_of_attempts: int
    max_bucket_size: int
    bucket_size: int
    refil_interval: int
    valid_from: int
    valid_until: int
    experiment_types: [str]

class Client:

    def __init__(self) -> None:
        self.robot_config = {}
        with open(f"robot.json", "r") as f:
            self.robot_config = json.load(f)

        can_baud_rate_config = self.robot_config["can-baud-rate"]
        can_baud_rate = None
        if can_baud_rate_config == 1:
            can_baud_rate = pyCandle.CAN_BAUD_1M
        elif can_baud_rate_config == 2:
            can_baud_rate = pyCandle.CAN_BAUD_2M
        elif can_baud_rate_config == 5:
            can_baud_rate = pyCandle.CAN_BAUD_5M
        elif can_baud_rate_config == 8:
            can_baud_rate = pyCandle.CAN_BAUD_8M
        else:
            raise RuntimeError(f"Invalid baudrate: {can_baud_rate_config}, should be 1, 2, 5 or 8")
        self.candle: pyCandle.Candle = \
            pyCandle.Candle(can_baud_rate, True, pyCandle.USB)
        self.all_motor_ids: list[int] = self.candle.ping(can_baud_rate)

        for joint in self._robot_joints():
            motor_id = joint["id"]
            if not motor_id in self.all_motor_ids:
                raise RuntimeError(f"Motor with id {motor_id} is not connected")

        for id in self.all_motor_ids:
            self.candle.addMd80(id)

        for md in self.candle.md80s:
            self.candle.controlMd80SetEncoderZero(md)
            self.candle.controlMd80Mode(md, pyCandle.IMPEDANCE)
            md.setImpedanceControllerParams(0.0, 0.0);
            self.candle.controlMd80Enable(md, True)

    def start_experiment(
        self,
        user_token: str = "",
        experiment_type: str = "",
        requested_time: float = 0.0,
        record: bool = False
    ) -> tuple[str, str]:
        self.candle.begin()
        return "", ""

    def stop_experiment(self, user_token: str = "") -> str:
        self._reset_motors()
        return ""

    def get_user_info(self, user_token: str = "") -> UserInfo:
        return UserInfo(
            user_name = "",
            max_experiment_length = 0.0,
            number_of_attempts = 0,
            max_bucket_size = 0,
            bucket_size = 0,
            refil_interval = 0,
            valid_from = 0,
            valid_until = 0,
            experiment_types = []
        )

    def get_joint_names(self, session_token: str = "") -> list[str]:
        return list(map(lambda j: j["name"], self._robot_joints()))

    def get_position(self, session_token: str = "") -> list[float]:
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            pos = self._get_motor(motor_id).getPosition()
            pos_limit = joint["pos_limit"]
            if not -pos_limit < pos < pos_limit:
                self._raise_exception(f"PositionLimitViolation;value={pos}")
            res = res + [pos]
        return self._prepare_output_data(res)
 
    def get_velocity(self, session_token: str = "") -> list[float]:
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            vel = self._get_motor(motor_id).getVelocity()
            vel_limit = joint["vel_limit"]
            if not -vel_limit < vel < vel_limit:
                self._raise_exception(f"VelocityLimitViolation;value={vel}")
            res = res + [vel]
        return self._prepare_output_data(res)

    def get_torque(self, session_token: str = "") -> list[float]:
        res: list[float] = []
        for joint in self._robot_joints():
            motor_id = joint["id"]
            torque = self._get_motor(motor_id).getTorque()
            torque_limit = joint["torque_limit"]
            if not -torque_limit < torque < torque_limit:
                self._raise_exception(f"TorqueLimitViolation;value={torque}")
            res = res + [torque]
        return self._prepare_output_data(res)

    def set_position(self, positions: list[float | None] | float, session_token: str = ""):
        positions = self._prepare_actuator_data(positions)
        for position, joint in zip(positions, self._robot_joints()):
            if position is None:
                continue
            pos_limit = joint["pos_limit"]
            clamped_position = min(pos_limit, max(-pos_limit, position))
            self._get_motor(joint["id"]).setTargetPosition(clamped_position)
            if pos_limit < abs(position):
               print(f"Position clamped from {position} to {clamped_position}")

    def set_velocity(self, velocities: list[float | None] | float, session_token: str = ""):
        velocities = self._prepare_actuator_data(velocities)
        for velocity, joint in zip(velocities, self._robot_joints()):
            if velocity is None:
                continue
            vel_limit = joint["vel_limit"]
            clamped_velocity = min(vel_limit, max(-vel_limit, velocity))
            self._get_motor(joint["id"]).setTargetVelocity(clamped_velocity)
            if vel_limit < abs(velocity):
                print(f"Velocity clamped from {velocity} to {clamped_velocity}")

    def set_torque(self, torques: list[float | None] | float, session_token: str = ""):
        torques = self._prepare_actuator_data(torques)
        for torque, joint in zip(torques, self._robot_joints()):
            if torque is None:
                continue
            torque_limit = joint["torque_limit"]
            clamped_torque = min(torque_limit, max(-torque_limit, torque))
            self._get_motor(joint["id"]).setTargetTorque(clamped_torque)
            if torque_limit < abs(torque):
                print(f"Torque clamped from {torque} to {clamped_torque}")

    def set_impedance_controller_params(
        self,
        kp: float | list[float],
        kd: float | list[float],
        session_token: str = ""
    ):
        kp = self._prepare_actuator_data(kp)
        kd = self._prepare_actuator_data(kd)
        for p, d, joint in zip(kp, kd, self._robot_joints()):
            if p is None or d is None:
                continue
            motor = self._get_motor(joint["id"])
            motor.setImpedanceControllerParams(p, d)

    def _robot_joints(self):
        return self.robot_config["robot-joints"]

    def _get_motor(self, id: int) -> pyCandle.Md80:
        for md in self.candle.md80s:
            if md.getId() == id:
                return md
        self._raise_exception(f"No motor with id {id} connected")

    def _reset_motors(self):
        for motor in self.candle.md80s:
            pos = motor.getPosition()
            offset = round(pos / (2.0 * math.pi)) * (2.0 * math.pi)
            motor.setImpedanceControllerParams(0.05, 0.005)
            motor.setTargetTorque(0.0)
            motor.setTargetPosition(offset)
            motor.setTargetVelocity(0.0)
        time.sleep(2.0)
        self.candle.end()

    def _prepare_actuator_data(
        self,
        data: float | list[float | None]
    ) -> list[float | None]:
        if not isinstance(data, list):
            data = [data]
        for d in data:
            if d is not None:
                d = float(d)
        if len(data) != len(self._robot_joints()):
            self._raise_exception(f"Wrong number of arguments supplied; Expected {len(self._robot_joints())}, got {len(data)}")
        return data

    def _prepare_output_data(self, data: list[float]) -> list[float] | float:
        if len(data) == 1:
            return data[0]
        return data

    def _raise_exception(self, message):
        self._reset_motors()
        raise RuntimeError(message)
