import logging

import numpy as np

from lerobot.errors import DeviceNotConnectedError
from . import WidowX250S
from .widowX_250S_config import WidowX250SEndEffectorConfig
from ...model.kinematics import RobotKinematics

logger = logging.getLogger(__name__)


def ypr_to_rotation_matrix(self, yaw: float, pitch: float, roll: float) -> np.ndarray:
    """
    Convention: intrinsic ZYX (yaw-pitch-roll).
    """
    cx, cy, cz = np.cos([roll, pitch, yaw])
    sx, sy, sz = np.sin([roll, pitch, yaw])

    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx]
    ])


class WidowX250SEndEffector(WidowX250S):
    """
    WidowX250S adapter for end-effector space control.
    """

    config_class = WidowX250SEndEffectorConfig
    name = "widowx_250s_end_effector"

    def __init__(self, config: WidowX250SEndEffectorConfig):
        super().__init__(config)
        self.config = config

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

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
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

        rot_delta = np.eye(3)
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
                if "delta_pitch" in action or "delta_roll" in action or "delta_yaw" in action:
                    droll = action.get("delta_roll", 0.0)
                    dpitch = action.get("delta_pitch", 0.0)
                    dyaw = action.get("delta_yaw", 0.0)
                    rot_delta = ypr_to_rotation_matrix(self, dyaw, dpitch, droll)
                if "goto_home" in action and action["goto_home"] > 0.5:
                    # Reset current positions to home
                    self.current_ee_pos_metrics = None
                    self.current_joint_pos_metrics = None
                    self.send_to_home()
                    logger.info("Going to home position")
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                action = np.zeros(4, dtype=np.float32)

        if self.current_joint_pos_metrics is None:
            current_joint_pos_dict = self.bus.sync_read("Present_Position", normalize=True)
            current_joint_pos_metrics = self.normalized_joint_pos_to_metrics(current_joint_pos_dict)
            self.current_joint_pos_metrics = np.array([current_joint_pos_metrics[name] for name in self.motor_names_for_observations])

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
        # Apply rotation delta in the end-effector frame
        desired_ee_pos_metrics[:3, :3] = desired_ee_pos_metrics[:3, :3] @ rot_delta

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
            target_joint_pos_metrics[-1] = self.current_joint_pos_metrics[-1]

        self.current_ee_pos_metrics = desired_ee_pos_metrics.copy()
        self.current_joint_pos_metrics = target_joint_pos_metrics.copy()

        joint_metrics_dict = {key: target_joint_pos_metrics[i] for i, key in enumerate(self.motor_names_for_observations)}

        goal_pos = self.metrics_joint_pos_to_normalized(joint_metrics_dict)
        joint_action = {f"{motor}.pos": val for motor, val in goal_pos.items()}
        # Send joint space action to parent class
        return super().send_action(joint_action)

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
