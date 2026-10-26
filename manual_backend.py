"""Small, physics-driven manual-flight backend for Pegasus Simulator 5.1.

Commands are ENU altitude and yaw-relative forward/left velocity.  The backend
produces rotor angular speeds via Pegasus's allocation method; it never changes
the vehicle transform or velocity.  Call after SimulationApp has been created.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.backends import Backend


def _bounded_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector * min(1.0, maximum / max(norm, 1e-12))


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ManualFlightBackend(Backend):
    """Position-hold flight assistance with bounded manual velocity commands.

    ``set_velocity_command`` arguments use m/s, m/s, m/s and rad/s.  Refresh
    commands each UI update (not only key-down), and send zeros on key release or
    focus loss. ``takeoff`` specifies an absolute world Z coordinate in metres.
    ``ground_height`` is the drone BODY's resting Z coordinate on the floor,
    approximately 0.07 m for the stock Iris, not the height of the floor mesh.
    """

    def __init__(
        self,
        mass: float = 1.5,
        ground_height: float = 0.07,
        max_altitude: float = 8.0,
        horizontal_speed: float = 1.5,
        vertical_speed: float = 0.7,
        yaw_rate_limit: float = 0.6,
        command_timeout: float = 0.5,
        horizontal_bounds: Optional[tuple[float, float, float, float]] = None,
    ):
        super().__init__(None)
        self.mass = float(mass)
        self.ground_height = float(ground_height)
        self.max_altitude = float(max_altitude)
        self.horizontal_speed = float(horizontal_speed)
        self.vertical_speed = float(vertical_speed)
        self.yaw_rate_limit = float(yaw_rate_limit)
        self.command_timeout = float(command_timeout)
        self.horizontal_bounds = horizontal_bounds  # xmin, xmax, ymin, ymax
        if self.mass <= 0 or self.max_altitude <= self.ground_height + 0.3:
            raise ValueError("Invalid mass or flight altitude range")
        self.kp = np.array([2.8, 2.8, 4.0], dtype=float)
        self.kd = np.array([3.6, 3.6, 4.0], dtype=float)
        self.kr = np.array([3.0, 3.0, 1.0], dtype=float)
        self.kw = np.array([0.45, 0.45, 0.25], dtype=float)
        self.reset()

    def reset(self):
        self.mode = "GROUNDED"
        self.fault = ""
        self._ready = False
        self._reference_ready = False
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._w = np.zeros(3)
        self._rotation = np.eye(3)
        self._target = np.zeros(3)
        self._yaw_target = 0.0
        self._yaw = 0.0
        self._velocity_reference = np.zeros(3)
        self._command = np.zeros(4)
        self._clock = 0.0
        self._command_time = -math.inf
        self._z_integral = 0.0
        self._touchdown_time = 0.0
        self._takeoff_altitude = 2.0
        self._last_thrust = 0.0
        self._last_torque = np.zeros(3)
        self.input_ref = [0.0] * 4

    def start(self):
        # Timeline stop/play must not replay a held key or an old flight target.
        self.reset()

    def stop(self):
        self.reset()

    def update_sensor(self, sensor_type: str, data):
        pass

    def update_graphical_sensor(self, sensor_type: str, data):
        pass

    def update_state(self, state):
        p = np.asarray(state.position, dtype=float).copy()
        v = np.asarray(state.linear_velocity, dtype=float).copy()
        w = np.asarray(state.angular_velocity, dtype=float).copy()
        q = np.asarray(state.attitude, dtype=float).copy()  # xyzw, ENU/FLU
        if not all(np.all(np.isfinite(a)) for a in (p, v, w, q)) or np.linalg.norm(q) < 0.1:
            self.emergency_stop("Non-finite physics state")
            return
        self._p, self._v, self._w = p, v, w
        self._rotation = Rotation.from_quat(q).as_matrix()
        self._yaw = math.atan2(self._rotation[1, 0], self._rotation[0, 0])
        self._ready = True
        if not self._reference_ready:
            self._capture_reference()

    def _capture_reference(self):
        self._target = self._p.copy()
        self._yaw_target = self._yaw
        self._velocity_reference[:] = 0
        self._reference_ready = True

    def input_reference(self):
        return self.input_ref

    def set_velocity_command(self, forward=0.0, left=0.0, up=0.0, yaw_rate=0.0):
        command = np.asarray([forward, left, up, yaw_rate], dtype=float)
        if not np.all(np.isfinite(command)):
            raise ValueError("Flight command must contain finite numbers")
        command[:2] = _bounded_norm(command[:2], self.horizontal_speed)
        command[2] = np.clip(command[2], -self.vertical_speed, self.vertical_speed)
        command[3] = np.clip(command[3], -self.yaw_rate_limit, self.yaw_rate_limit)
        self._command = command
        self._command_time = self._clock
        if self.mode in ("HOLD", "FLYING", "TAKEOFF") and np.linalg.norm(command) > 1e-5:
            self.mode = "FLYING"

    def takeoff(self, altitude=2.0):
        if self.fault:
            return False
        if not math.isfinite(float(altitude)):
            raise ValueError("Takeoff altitude must be finite")
        self._takeoff_altitude = float(np.clip(altitude, self.ground_height + 0.5, self.max_altitude))
        self._command[:] = 0
        self._z_integral = 0.0
        self._touchdown_time = 0.0
        if self._ready:
            self._capture_reference()
        self.mode = "TAKEOFF"
        return True

    def hold(self):
        self._command[:] = 0
        if self.mode in ("GROUNDED", "FAULT"):
            return
        if self._ready:
            self._capture_reference()
        self.mode = "HOLD"

    def land(self):
        self._command[:] = 0
        if self.mode in ("GROUNDED", "FAULT"):
            return
        if self._ready:
            self._capture_reference()
        self._z_integral = 0.0
        self._touchdown_time = 0.0
        self.mode = "LANDING"

    def emergency_stop(self, reason="Emergency motor stop"):
        """Disarm immediately; gravity still acts. Use timeline reset after a crash."""
        self._command[:] = 0
        self.input_ref = [0.0] * 4
        self._last_thrust = 0.0
        self.fault = str(reason)
        self.mode = "FAULT"

    def _advance_reference(self, dt):
        desired_velocity = np.zeros(3)
        yaw_rate = 0.0
        if self.mode == "TAKEOFF":
            delta = self._takeoff_altitude - self._target[2]
            # Brake as the altitude reference approaches its destination. A
            # delta/dt bang-bang reference would oscillate after slew limiting.
            desired_velocity[2] = np.clip(1.5 * delta, -self.vertical_speed, self.vertical_speed)
            if abs(self._p[2] - self._takeoff_altitude) < 0.12 and abs(self._v[2]) < 0.2:
                self.mode = "HOLD"
                self._target[2] = self._takeoff_altitude
                desired_velocity[:] = 0
        elif self.mode == "LANDING":
            # Target a little below the resting body origin so contact is reliable.
            desired_velocity[2] = -0.35 if self._target[2] > self.ground_height - 0.06 else 0.0
        else:
            if self._clock - self._command_time > self.command_timeout:
                self._command[:] = 0
            forward, left, up, yaw_rate = self._command
            c, s = math.cos(self._yaw), math.sin(self._yaw)
            desired_velocity = np.array([c * forward - s * left, s * forward + c * left, up])
            self.mode = "FLYING" if np.linalg.norm(self._command) > 1e-5 else "HOLD"

        # Slew velocity commands to keep a small drone comfortable and controllable.
        change = desired_velocity - self._velocity_reference
        change[:2] = _bounded_norm(change[:2], 1.4 * dt)
        change[2] = np.clip(change[2], -1.0 * dt, 1.0 * dt)
        self._velocity_reference += change
        self._target += self._velocity_reference * dt
        self._yaw_target = _wrap(self._yaw_target + yaw_rate * dt)

        # A wall cannot accumulate a far-away target that would launch the drone
        # after it is freed. No horizontal integral is used for the same reason.
        offset = self._target[:2] - self._p[:2]
        self._target[:2] = self._p[:2] + _bounded_norm(offset, 0.8)
        self._target[2] = np.clip(self._target[2], self._p[2] - 0.7, self._p[2] + 0.7)
        z_min = self.ground_height - 0.06 if self.mode == "LANDING" else self.ground_height + 0.25
        self._target[2] = np.clip(self._target[2], z_min, self.max_altitude)
        if (self._target[2] >= self.max_altitude and self._velocity_reference[2] > 0) or (
            self._target[2] <= z_min and self._velocity_reference[2] < 0
        ):
            self._velocity_reference[2] = 0.0
        if self.horizontal_bounds is not None:
            xmin, xmax, ymin, ymax = self.horizontal_bounds
            previous = self._target[:2].copy()
            self._target[:2] = np.clip(previous, [xmin, ymin], [xmax, ymax])
            self._velocity_reference[:2][previous != self._target[:2]] = 0.0
        return yaw_rate

    def update(self, dt: float):
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            return
        self._clock += dt
        if not self._ready or self.mode in ("GROUNDED", "FAULT"):
            self.input_ref = [0.0] * 4
            return
        # Physics should run at 120 Hz; bounded control integration also handles
        # a single long callback without advancing the reference metres at once.
        dt = min(dt, 0.05)
        yaw_rate = self._advance_reference(dt)
        if self._rotation[2, 2] < math.cos(math.radians(75.0)):
            self.emergency_stop("Drone tipped over; stop and restart the simulation")
            return

        if self.mode == "LANDING":
            contact = self._p[2] <= self.ground_height + 0.10 and abs(self._v[2]) < 0.18
            self._touchdown_time = self._touchdown_time + dt if contact else 0.0
            if self._touchdown_time > 0.55:
                self.mode = "GROUNDED"
                self.input_ref = [0.0] * 4
                self._last_thrust = 0.0
                self._z_integral = 0.0
                return

        error = self._target - self._p
        # Only a small bounded vertical integral compensates model mass error.
        # Freeze it on the ground/landing and during large position errors.
        if self.mode != "LANDING" and self._p[2] > self.ground_height + 0.15 and abs(error[2]) < 0.4:
            self._z_integral = float(np.clip(self._z_integral + error[2] * dt, -1.0, 1.0))
        acceleration = self.kp * error + self.kd * (self._velocity_reference - self._v)
        acceleration[2] += 0.45 * self._z_integral
        acceleration[:2] = _bounded_norm(acceleration[:2], 2.5)
        acceleration[2] = np.clip(acceleration[2], -3.0, 4.0)
        force = self.mass * (acceleration + np.array([0.0, 0.0, 9.81]))
        # Bound tilt to twenty degrees relative to the vertical force component.
        force[:2] = _bounded_norm(force[:2], force[2] * math.tan(math.radians(20)))
        desired_z = force / max(float(np.linalg.norm(force)), 1e-9)
        heading = np.array([math.cos(self._yaw_target), math.sin(self._yaw_target), 0.0])
        desired_y = np.cross(desired_z, heading)
        desired_y /= max(float(np.linalg.norm(desired_y)), 1e-9)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        r = self._rotation
        skew_error = 0.5 * (desired_rotation.T @ r - r.T @ desired_rotation)
        attitude_error = np.array([skew_error[2, 1], skew_error[0, 2], skew_error[1, 0]])
        desired_body_rate = r.T @ np.array([0.0, 0.0, yaw_rate])
        torque = -self.kr * attitude_error - self.kw * (self._w - desired_body_rate)
        torque = np.clip(torque, [-1.4, -1.4, -0.45], [1.4, 1.4, 0.45])
        thrust = float(np.clip(force @ r[:, 2], 0.0, 2.0 * self.mass * 9.81))
        self._last_thrust = thrust
        self._last_torque = torque
        if self.vehicle is not None:
            speeds = np.asarray(self.vehicle.force_and_torques_to_velocities(thrust, torque), dtype=float)
            if not np.all(np.isfinite(speeds)):
                self.emergency_stop("Motor allocation returned non-finite speeds")
                return
            self.input_ref = speeds.tolist()

    def telemetry(self):
        """JSON-ready controller/physics snapshot for the UI and acceptance log."""
        return {
            "mode": self.mode,
            "ready": self._ready,
            "fault": self.fault,
            "time_s": self._clock,
            "position_m": self._p.tolist(),
            "velocity_m_s": self._v.tolist(),
            "target_m": self._target.tolist(),
            "yaw_deg": math.degrees(self._yaw),
            "target_yaw_deg": math.degrees(self._yaw_target),
            "tilt_deg": math.degrees(math.acos(float(np.clip(self._rotation[2, 2], -1, 1)))),
            "motor_rad_s": list(self.input_ref),
            "thrust_N": self._last_thrust,
            "torque_Nm": self._last_torque.tolist(),
        }

