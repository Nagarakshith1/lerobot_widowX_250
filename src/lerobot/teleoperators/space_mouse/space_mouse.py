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


from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

import pyspacemouse
from pynput import keyboard

from lerobot.errors import DeviceNotConnectedError
from .space_mouse_config import SpaceMouseConfig
from ..teleoperator import Teleoperator

logger = logging.getLogger(__name__)

import threading


@dataclass(frozen=True)
class _SpaceMouseEvent:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    left_button: bool = False
    right_button: bool = False


class SpaceMouse(Teleoperator):
    config_class = SpaceMouseConfig
    name = "spacemouse"

    def __init__(self, config: SpaceMouseConfig):
        super().__init__(config)
        self.config = config
        self._connected: bool = False

        # SpaceMouse state
        self._thread = None
        self._last_evt = None
        self._lock = threading.Lock()

        # Keyboard hotkeys
        self._orientation_lock = False
        self._translation_lock = False
        self.gripper_state = 1.0  # 0=close, 1=hold, 2=open
        self._kb_listener = keyboard.Listener(on_press=self._on_key_press) if config.enable_hotkeys else None

    def _poll_loop(self):
        while self.is_connected:
            evt = pyspacemouse.read()
            if evt is not None:
                with self._lock:
                    self._last_evt = evt
            # small sleep prevents 100% CPU; HID polling is ~125Hz anyway
            time.sleep(0.001)

    def get_latest_event(self) -> _SpaceMouseEvent:
        with self._lock:
            if self._last_evt is None:
                return _SpaceMouseEvent()

            return _SpaceMouseEvent(x=self._last_evt.x, y=self._last_evt.y, z=self._last_evt.z,
                                    yaw=self._last_evt.yaw, pitch=self._last_evt.pitch, roll=self._last_evt.roll,
                                    left_button=bool(self._last_evt.buttons[0]), right_button=bool(self._last_evt.buttons[1]))

    def _on_key_press(self, key):
        try:
            if key.char == 't':
                self._translation_lock = not self._translation_lock
                logger.info(f"Translation lock: {self._translation_lock}")
                if self._translation_lock and self._orientation_lock:
                    logger.warning(f"All motions locked!")
            elif key.char == 'r':
                self._orientation_lock = not self._orientation_lock
                logger.info(f"Orientation lock: {self._orientation_lock}")
                if self._translation_lock and self._orientation_lock:
                    logger.warning(f"All motions locked!")
            elif key.char == 'g':
                # Initially set to hold and on latter presses cycle toggle between close and open
                if self.gripper_state == 1.0:
                    self.gripper_state = 0.0
                elif self.gripper_state == 0.0:
                    self.gripper_state = 2.0
                elif self.gripper_state == 2.0:
                    self.gripper_state = 0.0
        except AttributeError:
            pass

    @property
    def action_features(self) -> dict:
        return {
            "delta_x": float,
            "delta_y": float,
            "delta_z": float,
            "delta_pitch": float,
            "delta_roll": float,
            "gripper": float,
        }

    @property
    def feedback_features(self) -> dict:
        # No active haptics/vibration on SpaceMouse; keep as empty dict
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if pyspacemouse is None:
            raise ImportError(
                "pyspacemouse is not available. Install with `pip install pyspacemouse hidapi easyhid`."
            )
        ok = pyspacemouse.open()
        if not ok:
            raise RuntimeError(
                "Failed to open SpaceMouse. On Linux, check udev permissions (vendor id 256f)."
            )
        self._connected = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        logger.info("SpaceMouse connected.")
        if self._kb_listener:
            self._kb_listener.daemon = True
            self._kb_listener.start()
            logger.info("Keyboard listener started for hotkeys.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _apply_deadzone(self, v: float) -> float:
        dz = self.config.deadzone
        if abs(v) < dz:
            return 0.0
        # re-scale out the deadzone so small motions become responsive again
        # map [dz..1] -> [0..1]
        s = (abs(v) - dz) / (1.0 - dz)
        s = max(0.0, min(1.0, s))
        return s if v > 0 else -s

    def _apply_expo(self, v: float) -> float:
        e = self.config.expo
        if e <= 1.0:
            return v
        return (abs(v) ** e) * (1.0 if v >= 0 else -1.0)

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        evt = self.get_latest_event()

        dx, dy, dz = 0.0, 0.0, 0.0
        if not self._translation_lock:
            tx = self._apply_expo(self._apply_deadzone(evt.x))
            ty = self._apply_expo(self._apply_deadzone(evt.y))
            tz = self._apply_expo(self._apply_deadzone(evt.z))

            # Gains and per-axis scalers
            dx = tx * self.config.gain_xyz
            dy = ty * self.config.gain_xyz
            dz = tz * self.config.gain_xyz

        droll, dpitch = 0.0, 0.0
        if not self._orientation_lock:
            rp = self._apply_expo(self._apply_deadzone(evt.pitch))
            rr = self._apply_expo(self._apply_deadzone(evt.roll))
            # Flip pitch/roll to match end-effector frame (x-forward, y-left, z-up)
            droll = rp * self.config.gain_roll
            dpitch = rr * self.config.gain_pitch

        return {
            "delta_x": dx,
            "delta_y": dy,
            "delta_z": dz,
            "delta_pitch": dpitch,
            "delta_roll": droll,
            "gripper": self.gripper_state,
            "goto_home": 1.0 if evt.left_button and evt.right_button else 0.0
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        return None

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        try:
            pyspacemouse.close()
        finally:
            self._connected = False
            if self._thread:
                self._thread.join(timeout=1.0)
            logger.info("SpaceMouse disconnected.")

        try:
            self._kb_listener.stop()
            self._kb_listener = None
            logger.info("Keyboard listener stopped.")
        except Exception as e:
            logger.warning(f"Failed to stop keyboard listener: {e}")
