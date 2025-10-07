#!/usr/bin/env python

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

from dataclasses import dataclass
from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("space_mouse")
@dataclass
class SpaceMouseConfig(TeleoperatorConfig):
    # Linear gain [m/step] and yaw gain [rad/step]
    gain_xyz: float = 0.3
    gain_pitch: float = 0.05
    gain_roll: float = 0.2
    # Deadzone on raw normalized axes (0..1)
    deadzone: float = 0.03
    # Exponential response (value^expo * sign)
    expo: float = 1.7
    enable_hotkeys: bool = True