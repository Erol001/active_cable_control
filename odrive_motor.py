# odrive_motor.py

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from config import (MAX_RPM_SLOW, MAX_RPM_FAST, VEL_RAMP_RATE,
                    ACCEL_LIMIT, DECEL_LIMIT, ACCEL_LIMIT_FAST,
                    DECEL_LIMIT_FAST, CURRENT_LIM, TORQUE_LIM)

class OdriveMotor:
    def __init__(self):
        self.odrv = None
        self.axis = None
        self.motor_turns = 0.0
        self._connected = False

    def connect(self):
        print("Connecting to ODrive...")
        try:
            self.odrv = odrive.find_any(timeout=3)
            self.axis = self.odrv.axis0
            self._connected = True
            print(f"Connected. Vbus: {self.odrv.vbus_voltage:.2f}V")
        except Exception as e:
            print(f"ODrive not found: {e}")
            self._connected = False

    def enable(self):
        if not self._connected:
            return
        try:
            # Forcer IDLE d'abord pour effacer tout état résiduel
            self.axis.requested_state = AxisState.IDLE
            import time; time.sleep(0.5)

            # Motor limits
            self.axis.config.motor.current_soft_max = CURRENT_LIM
            self.axis.config.torque_soft_max = TORQUE_LIM

            # Trajectory config
            self.axis.controller.config.vel_ramp_rate = VEL_RAMP_RATE
            self.axis.trap_traj.config.accel_limit = ACCEL_LIMIT
            self.axis.trap_traj.config.decel_limit = DECEL_LIMIT
            self.axis.trap_traj.config.vel_limit = MAX_RPM_FAST / 60

            # Control mode
            self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
            self.axis.controller.config.input_mode = InputMode.TRAP_TRAJ

            # Figer sur la position actuelle
            self.axis.controller.input_pos = self.axis.pos_estimate

        except Exception as e:
            print(f"Enable error: {e}")
            self._connected = False

    def enable_fast(self):
        if not self._connected:
            return
        try:
            self.axis.config.motor.current_soft_max = CURRENT_LIM
            self.axis.config.torque_soft_max = TORQUE_LIM
            self.axis.trap_traj.config.accel_limit = ACCEL_LIMIT_FAST
            self.axis.trap_traj.config.decel_limit = DECEL_LIMIT_FAST
            self.axis.trap_traj.config.vel_limit = MAX_RPM_FAST / 60
            self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
            self.axis.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.axis.controller.input_pos = self.axis.pos_estimate
        except Exception as e:
            print(f"Enable fast error: {e}")
            self._connected = False

    def disable(self):
        if not self._connected:
            return
        try:
            self.axis.controller.input_torque = 0
            self.axis.controller.input_vel = 0
            self.axis.requested_state = AxisState.IDLE
        except Exception as e:
            print(f"Disable error: {e}")
            self._connected = False

    def set_target_turns(self, target_turns, slow_mode=False, custom_rpm=None):
        if not self._connected:
            return
        try:
            if custom_rpm is not None:
                max_rps = custom_rpm / 60
            else:
                max_rpm = MAX_RPM_SLOW if slow_mode else MAX_RPM_FAST
                max_rps = max_rpm / 60
            self.axis.trap_traj.config.vel_limit = max_rps
            self.axis.controller.input_pos = target_turns
        except Exception as e:
            print(f"Set target error: {e}")
            self._connected = False

    def set_traj_params(self, accel=None, decel=None, max_rpm=None):
        if not self._connected:
            return
        try:
            if accel is not None:
                self.axis.trap_traj.config.accel_limit = accel
            if decel is not None:
                self.axis.trap_traj.config.decel_limit = decel
            if max_rpm is not None:
                self.axis.trap_traj.config.vel_limit = max_rpm / 60
        except Exception as e:
            print(f"Set traj params error: {e}")

    def get_motor_turns(self):
        if self._connected:
            try:
                self.motor_turns = self.axis.pos_estimate
            except Exception as e:
                print(f"Get turns error: {e}")
                self._connected = False
        return self.motor_turns

    def get_rpm(self):
        if self._connected:
            try:
                return self.axis.vel_estimate * 60
            except Exception as e:
                print(f"Get RPM error: {e}")
                self._connected = False
        return 0.0

    def get_current(self):
        if self._connected:
            try:
                return self.axis.motor.foc.Iq_measured
            except Exception as e:
                print(f"Get current error: {e}")
        return 0.0

    def is_connected(self):
        return self._connected