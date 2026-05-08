import time
from dataclasses import dataclass
from typing import Any

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


@dataclass
class PiperMotorsBusConfig:
    can_name: str
    motors: dict[str, tuple[int, str]]
    robot_model: str = ArmModel.PIPER
    firmware_version: str = PiperFW.DEFAULT
    interface: str = "socketcan"
    bitrate: int = 1_000_000
    gripper_force: float = 1.0
    enable_check_can: bool = True


class PiperMotorsBus:
    """
    pyAgxArm adapter used by the Piper leader/follower integrations.

    The public LeRobot-facing API keeps joint values in SI-style units:
    six arm joints in radians plus one gripper width in meters.
    """

    def __init__(self, config: PiperMotorsBusConfig):
        self.config = config
        self.motors = config.motors
        self.init_joint_position = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0]
        self.safe_disable_position = [0.0, 0.0, 0.0, 0.0, 0.52, 0.0, 0.0]
        self._is_connected = False
        self._is_calibrated = False
        self._gripper = None

        arm_config = create_agx_arm_config(
            robot=config.robot_model,
            firmeware_version=config.firmware_version,
            interface=config.interface,
            channel=config.can_name,
            bitrate=config.bitrate,
            enable_check_can=config.enable_check_can,
            auto_connect=False,
        )
        self.piper = AgxArmFactory.create_arm(arm_config)
        self._gripper = self.piper.init_effector(self.piper.OPTIONS.EFFECTOR.AGX_GRIPPER)

    @property
    def motor_names(self) -> list[str]:
        return list(self.motors.keys())

    @property
    def motor_models(self) -> list[str]:
        return [model for _, model in self.motors.values()]

    @property
    def motor_indices(self) -> list[int]:
        return [idx for idx, _ in self.motors.values()]

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def _get_enable_status(self) -> list[bool]:
        return list(self.piper.get_joints_enable_status_list())

    def _wait_for_enable_state(self, enable: bool, timeout_s: float = 5.0) -> bool:
        start_time = time.monotonic()
        while time.monotonic() - start_time <= timeout_s:
            enable_list = self._get_enable_status()
            if enable_list and all(bool(status) == enable for status in enable_list):
                return True
            time.sleep(0.1)
        return False

    def _set_enable_state(self, enable: bool, timeout_s: float = 5.0) -> bool:
        start_time = time.monotonic()
        while time.monotonic() - start_time <= timeout_s:
            if enable:
                self.piper.enable()
            else:
                self.piper.disable()

            if self._wait_for_enable_state(enable=enable, timeout_s=0.5):
                return True

        return False

    def connect(self, enable: bool) -> bool:
        """Connect/disconnect the CAN driver and enable/disable the Piper arm."""
        if enable:
            if not self.piper.is_connected():
                self.piper.connect()
            enabled = self._set_enable_state(enable=True)
            if self._gripper is not None:
                self._gripper.move_gripper_m(value=0.0, force=self.config.gripper_force)
            self._is_connected = enabled
            return enabled

        if self.piper.is_connected():
            if self._gripper is not None:
                self._gripper.disable_gripper()
            disabled = self._set_enable_state(enable=False)
            self.piper.disconnect()
        else:
            disabled = True

        self._is_connected = False
        self._is_calibrated = False
        return disabled

    def set_calibration(self):
        return

    def revert_calibration(self):
        return

    def apply_calibration(self):
        """Move to the configured initial position."""
        self.write(target_joint=self.init_joint_position)
        self._is_calibrated = True

    def apply_calibration_master(self):
        """Move the leader arm to the configured initial position."""
        self.write(target_joint=self.init_joint_position)
        self._is_calibrated = True

    def write(self, target_joint: list[float]):
        """
        Joint control.

        Args:
            target_joint: [joint_1..joint_6, gripper_width], with joints in
                radians and gripper width in meters.
        """
        if len(target_joint) != 7:
            raise ValueError(f"Expected 7 values ([6 joints + gripper]), got {len(target_joint)}.")
        if not self.piper.is_connected():
            raise ConnectionError("Piper arm is not connected.")

        joints = [float(value) for value in target_joint[:6]]
        gripper_width = max(0.0, float(target_joint[6]))

        self.piper.move_j(joints)
        if self._gripper is not None:
            self._gripper.move_gripper_m(value=gripper_width, force=self.config.gripper_force)

    def read(self) -> dict[str, float]:
        """
        Read current arm state.

        Returns joints in radians and gripper width in meters.
        """
        joint_msg = self.piper.get_joint_angles()
        if joint_msg is None:
            raise RuntimeError("No Piper joint feedback has been received yet.")

        state: dict[str, float] = {
            f"joint_{idx}": float(value) for idx, value in enumerate(joint_msg.msg, start=1)
        }
        state["gripper"] = 0.0

        if self._gripper is not None:
            gripper_msg: Any = self._gripper.get_gripper_status()
            if gripper_msg is not None and getattr(gripper_msg.msg, "mode", "width") == "width":
                state["gripper"] = float(gripper_msg.msg.value)

        return state

    def safe_disconnect(self):
        """Move to the configured safe disconnect position."""
        self.write(target_joint=self.safe_disable_position)

    def safe_disconnect_master(self):
        """Move the leader arm to the configured safe disconnect position."""
        self.write(target_joint=self.safe_disable_position)
