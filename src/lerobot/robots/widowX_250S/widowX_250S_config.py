# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from ..config import RobotConfig


@RobotConfig.register_subclass("widowx_250s")
@dataclass
class WidowX250SConfig(RobotConfig):
    port: str  # Port to connect to the arm

    disable_torque_on_disconnect: bool = True
    max_relative_target: float | None = 10.

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    closed_gripper_pos: float = 40
    open_gripper_pos: float = 70

    velocity_tps: int = 50  # ticks per second
    acceleration_tps2: int = 100  # ticks per second squared


@RobotConfig.register_subclass("widowx_250s_end_effector")
@dataclass
class WidowX250SEndEffectorConfig(WidowX250SConfig):
    """Configuration for the WidowX250SEndEffector robot."""
    # End-effector frame name in the URDF
    target_frame_name: str = "ee_arm_link"

    urdf_path: str | None = os.path.join(os.path.dirname(__file__), "widowX_250S.urdf")

    # Default bounds for the end-effector position (in meters)
    end_effector_bounds: dict[str, list[float]] = field(
        default_factory=lambda: {
            "min": [-0.2, -0.3, -0.2],  # min x, y, z
            "max": [0.5, 0.3, 0.5],  # max x, y, z
        }
    )

    # Defined in meters
    end_effector_step_sizes: dict[str, float] = field(
        default_factory=lambda: {
            "x": 0.02,
            "y": 0.02,
            "z": 0.02,
        }
    )
