import logging
import time
from functools import cached_property
from typing import Any

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.constants import OBS_STATE
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DynamixelMotorsBus,
    OperatingMode,
)
from numpy import ndarray
from rerun import disconnect

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .widowX_250S_config import WidowX250SConfig
from ...model.kinematics import RobotKinematics

logger = logging.getLogger(__name__)


class WidowX250S(Robot):
    """
    WidowX250 adapter
    """

    config_class = WidowX250SConfig
    name = "widowX250S"

    def __init__(self, config: WidowX250SConfig):
        super().__init__(config)
        self.config = config

        # --- Motors bus -----------------------------------------------------
        # Default mapping for a 6-DoF WidowX arm.
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "waist": Motor(1, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "shoulder": Motor(2, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "shoulder_shadow": Motor(3, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "elbow": Motor(4, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "elbow_shadow": Motor(5, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "forearm_roll": Motor(6, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "wrist_angle": Motor(7, "xm430-w350", MotorNormMode.RANGE_M100_100),
                "wrist_rotate": Motor(8, "xl430-w250", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(9, "xl430-w250", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )

        # Abusing the metrics name as we use degrees and not radians for angles
        # Obtained from https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html
        self.joint_limits_in_metrics = {
            "waist": (-180, 180),
            "shoulder": (-108, 114),
            "shoulder_shadow": (-108, 114),
            "elbow": (-123, 92),
            "elbow_shadow": (-123, 92),
            "forearm_roll": (-180, 180),
            "wrist_angle": (-100, 123),
            "wrist_rotate": (-180, 180),
            "gripper": (30, 74),
        }
        self.joint_names_for_kinematics = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate", "gripper"]

        # --- Cameras --------------------------------------------------------
        self.cameras = make_cameras_from_configs(config.cameras)
        # Initialize the kinematics module for the widowX250S robot
        if self.config.urdf_path is None:
            raise ValueError(
                "urdf_path must be provided in the configuration for end-effector control. "
            )

        self.kinematics = RobotKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
        )

        # Store the bounds for end-effector position
        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos_metrics = None
        self.current_joint_pos_metrics = None

    # --------------------------- Features -----------------------------------
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # ---------------------------- Lifecycle ---------------------------------
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()

        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        self.send_to_home()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        """Interactive basic calibration.

        * Sets Extended Position mode for arm joints; Current+Position for gripper
        * Guides the user to place mid-range and sweeps to record joint ranges
        * Persists calibration to the bus' calibration path
        """
        user_input = input(
            f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
        )
        if user_input.strip().lower() != "c":
            logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
            self.bus.write_calibration(self.calibration)
            return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()

        # Configure operating modes before recording ranges
        for motor in self.bus.motors:
            if motor == "gripper":
                self.bus.write("Operating_Mode", motor, OperatingMode.CURRENT_POSITION.value)
            else:
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        input("Move robot to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        # Motors that may reasonably be treated as full-turn during range capture
        full_turn_motors = [m for m in ["waist", "forearm_roll", "wrist_rotate"] if m in self.bus.motors]
        unknown_range_motors = [m for m in self.bus.motors if m not in full_turn_motors]
        print(
            f"Move all joints except {full_turn_motors} sequentially through their entire "
            "ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        for motor in full_turn_motors:
            # Allow wrap-around; use full register span
            range_mins[motor] = 0
            range_maxes[motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        """Set operating modes/limits after connect (torque disabled)."""
        with self.bus.torque_disabled():
            self.bus.configure_motors()

            # Extended position for all arm joints
            for motor in self.bus.motors:
                if motor == "gripper":
                    continue
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

            # Current+Position for gripper for gentle grasping
            self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)
            for motor in self.bus.motors:
                self.bus.write("Profile_Velocity", motor, self.config.velocity_tps)
                self.bus.write("Profile_Acceleration", motor, self.config.acceleration_tps2)

    # ------------------------------ IO --------------------------------------
    def get_observation(self) -> dict[str, Any]:
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}

        # Read arm positions (ticks)
        start = time.perf_counter()
        obs_state = self.bus.sync_read("Present_Position")
        obs_dict[OBS_STATE] = obs_state
        obs_dict.update({f"{motor}.pos": val for motor, val in obs_state.items()})
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration
        parameter `max_relative_target`. In this case, the action sent differs from the
        original action. Thus, this function always returns the action actually sent.

        Args:
            action (dict[str, float]): The goal positions for the motors, keyed by
                "<motor>.pos".
        Returns:
            dict[str, float]: The action sent to the motors, potentially clipped,
                with keys "<motor>.pos".
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Extract motor->goal mapping from "<motor>.pos" keys
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Clip goals if too far from present positions (smoother teleop/BC playback)
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal positions
        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def send_action_deltas(self, action: dict[str, float]) -> dict[str, float]:
        """Command action as deltas to the end-effector position.

        The relative action magnitude may be clipped depending on the configuration
        parameter `max_relative_target`. In this case, the action sent differs from the
        original action. Thus, this function always returns the action actually sent.

        Args:
            action (dict[str, float]): The goal positions for the motors, keyed by
                "<motor>.pos".
        Returns:
            dict[str, float]: The action sent to the motors, potentially clipped,
                with keys "<motor>.pos".
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        import numpy as np
        # Convert action to numpy array if not already
        if isinstance(action, dict):
            if all(k in action for k in ["delta_x", "delta_y", "delta_z"]):
                delta_ee = np.array(
                    [
                        action["delta_x"] * self.config.end_effector_step_sizes["x"],
                        action["delta_y"] * self.config.end_effector_step_sizes["y"],
                        action["delta_z"] * self.config.end_effector_step_sizes["z"],
                        ],
                    dtype=np.float32,
                )
                if "gripper" not in action:
                    action["gripper"] = [1.0]
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                action = np.zeros(4, dtype=np.float32)

        if self.current_joint_pos_metrics is None:
            current_joint_pos_dict = self.bus.sync_read("Present_Position", normalize=True)
            current_joint_pos_metrics = self.normalized_joint_pos_to_metrics(current_joint_pos_dict)
            self.current_joint_pos_metrics = np.array([current_joint_pos_metrics[name] for name in self.joint_names_for_kinematics])

        # Calculate current end-effector position using forward kinematics
        if self.current_ee_pos_metrics is None:
            self.current_ee_pos_metrics = self.kinematics.forward_kinematics(self.current_joint_pos_metrics)

        # Set desired end-effector position by adding delta
        desired_ee_pos_metrics = np.eye(4)
        desired_ee_pos_metrics[:3, :3] = self.current_ee_pos_metrics[:3, :3]  # Keep orientation

        # Add delta to position and clip to bounds
        desired_ee_pos_metrics[:3, 3] = self.current_ee_pos_metrics[:3, 3] + action[:3]
        # if self.end_effector_bounds is not None:
        desired_ee_pos_metrics[:3, 3] = np.clip(
            desired_ee_pos_metrics[:3, 3],
            self.end_effector_bounds["min"],
            self.end_effector_bounds["max"],
        )

        # Compute inverse kinematics to get joint positions
        target_joint_pos_metrics = self.kinematics.inverse_kinematics(
            self.current_joint_pos_metrics, desired_ee_pos_metrics
        )

        # 0 is close, 1 is stay, 2 is same
        # For now set to min and max position
        if action[-1] < 1:
            target_joint_pos_metrics[-1] = self.config.closed_gripper_pos
        elif action[-1] > 1:
            target_joint_pos_metrics[-1] = self.config.open_gripper_pos
        else:
            target_joint_pos_metrics[-1]  = self.current_joint_pos_metrics[-1]


        self.current_ee_pos_metrics = desired_ee_pos_metrics.copy()
        self.current_joint_pos_metrics = target_joint_pos_metrics.copy()

        joint_metrics_dict = {key: target_joint_pos_metrics[i] for i, key in enumerate(self.joint_names_for_kinematics)}
        # Add shadow joints and gripper
        joint_metrics_dict["shoulder_shadow"] = joint_metrics_dict["shoulder"]
        joint_metrics_dict["elbow_shadow"] = joint_metrics_dict["elbow"]

        goal_pos = self.metrics_joint_pos_to_normalized(joint_metrics_dict)

        # Send goal positions
        self.bus.sync_write("Goal_Position", goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
        return {}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.send_to_home()

        # Move to calibration file
        disconnect_pos_ticks = {
            "waist": 2017,
            "shoulder": 843,
            "shoulder_shadow": 858,
            "elbow": 3124,
            "elbow_shadow": 3065,
            "forearm_roll": 2016,
            "wrist_angle": 2623,
            "wrist_rotate": 2030,
            "gripper": self.calibration["gripper"].range_min
        }

        self.bus.sync_write("Goal_Position", disconnect_pos_ticks, normalize=False)
        time.sleep(3)  # Wait for the arm to reach the position

        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info(f"{self} disconnected.")

    def normalized_joint_pos_to_metrics(self, norm_pos: dict[str, float]) -> dict[str, float]:
        """Convert a normalized joint pose to degrees and mm.

        Args:
            norm_pos (dict[str, float]): Normalized joint positions, keyed by motor name.

        Returns:
            dict[str, float]: Joint positions in degrees, keyed by motor name.
        """
        # Unnormalize joint positions and map to degrees
        ids_values = self.bus._get_ids_values_dict(norm_pos)
        ids_values = self.bus._unnormalize(ids_values)
        motor_names_to_metrics = {self.bus._id_to_name(id_): value for id_, value in ids_values.items()}
        # Use the calibrations to convert ticks to degrees
        for motor_name, ticks in motor_names_to_metrics.items():
            min_deg, max_deg = self.joint_limits_in_metrics[motor_name]
            min_tick = self.bus.calibration[motor_name].range_min
            max_tick = self.bus.calibration[motor_name].range_max
            motor_names_to_metrics[motor_name] = min_deg + (ticks - min_tick) * (max_deg - min_deg) / (max_tick - min_tick)
        return motor_names_to_metrics

    def metrics_joint_pos_to_normalized(self, metrics_pos: dict[str, float]) -> dict[str, float]:
        """Convert a joint pose in degrees to normalized values.

        Args:
            metrics_pos (dict[str, float]): Joint positions in degrees, keyed by motor name.

        Returns:
            dict[str, float]: Normalized joint positions, keyed by motor name.
        """
        # Use the calibrations to convert degrees to ticks
        motor_names_to_ticks = {}
        for motor_name, deg in metrics_pos.items():
            min_deg, max_deg = self.joint_limits_in_metrics[motor_name]
            min_tick = self.bus.calibration[motor_name].range_min
            max_tick = self.bus.calibration[motor_name].range_max
            motor_names_to_ticks[motor_name] = min_tick + (deg - min_deg) * (max_tick - min_tick) / (max_deg - min_deg)
        ids_values = self.bus._get_ids_values_dict(motor_names_to_ticks)
        ids_values = self.bus._normalize(ids_values)
        norm_pos = {self.bus._id_to_name(id_): value for id_, value in ids_values.items()}
        return norm_pos

    def send_to_home(self) -> None:
        """Send the arm to a predefined home position."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        home_pos_ticks = {
            "waist": 2048 + self.calibration["waist"].homing_offset,
            "shoulder": 2048 + self.calibration["shoulder"].homing_offset,
            "shoulder_shadow": 2048 + self.calibration["shoulder_shadow"].homing_offset,
            "elbow": 2048 + self.calibration["elbow"].homing_offset,
            "elbow_shadow": 2048 + self.calibration["elbow_shadow"].homing_offset,
            "forearm_roll": 2048 + self.calibration["forearm_roll"].homing_offset,
            "wrist_angle": 2048 + self.calibration["wrist_angle"].homing_offset,
            "wrist_rotate": 2048 + self.calibration["wrist_rotate"].homing_offset,
            "gripper": self.calibration["gripper"].range_max
        }

        self.bus.sync_write("Goal_Position", home_pos_ticks, normalize=False)
        time.sleep(3)  # Wait for the arm to reach the position