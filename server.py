# server.py

import socket
import json
import threading
import time
import csv
import os
import math
import random
from odrive.enums import AxisState, ControlMode, InputMode
from config import HOST, PORT, CONTROL_FREQ, TRACKING_FREQ, GEAR_RATIO_FAST, GEAR_RATIO_SLOW
from hall_sensor import HallSensor
from spool_tracker import SpoolTracker
from data_logger import DataLogger
from odrive_motor import OdriveMotor
from switch_sensor import SwitchSensor
from servo_control import ServoController

CALIBRATION_RPMS = [100, 200, 400, 600, 800]
CALIBRATION_WAIT = 5

class WASPServer:
    def __init__(self):
        self.hall = HallSensor()
        self.spool = SpoolTracker()
        self.logger = DataLogger()
        self.motor = OdriveMotor()
        self.switch_sensor = SwitchSensor()
        self.servo = ServoController()

        self.target_length = 0.0
        self.z_ref = 0.0
        self.switch_active = False
        self.slow_mode = True
        self.custom_rpm = None
        self.running = False
        self._prev_switch = False
        self._total_turns_motor = 0.0
        self._prev_motor_turns = 0.0
        self._prev_motor_turns_dir = 0.0
        self._calibrating = False
        self._returning_to_zero = False
        self._rpm_test_active = False
        self._rpm_test_value = 0.0
        self._deploying = False
        self._tracking = False
        self._tracking_amplitude = 0.5
        self._tracking_freq = 0.5
        self._tracking_center = 2.0
        self._prbs_active = False
        self._steps_active = False
        self._step_status = ""

        self.servo_pos = 0.0
        self.servo_min = -1.0
        self.servo_max = 1.0

    def connect_motor(self):
        def _connect():
            try:
                self.motor.connect()
                self.motor.enable()
            except Exception as e:
                print(f"ODrive not connected: {e} — simulation mode")
        threading.Thread(target=_connect, daemon=True).start()

    def _re_enable_position_control(self):
        if not self.motor.is_connected():
            return
        try:
            current_pos = self.motor.axis.pos_estimate
            self.motor.enable()
            self.motor.axis.controller.input_pos = current_pos
            print("ODrive re-enabled in position control")
        except Exception as e:
            print(f"Re-enable error: {e}")
            self.motor._connected = False

    def _return_to_zero(self):
        if not self.motor.is_connected():
            return
        self._returning_to_zero = True
        print("Switch ON — returning to zero at 100 RPM")
        try:
            self.motor.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.motor.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
            self.motor.axis.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.motor.axis.trap_traj.config.vel_limit = 100 / 60
            self.motor.axis.trap_traj.config.accel_limit = 2.0
            self.motor.axis.trap_traj.config.decel_limit = 2.0
            self.motor.axis.controller.input_pos = 0

            while abs(self.motor.axis.pos_estimate) > 0.1:
                if not self.switch_active:
                    print("Switch OFF — aborting return to zero")
                    break
                time.sleep(0.1)

            print("Returned to zero")
        except Exception as e:
            print(f"Return to zero error: {e}")
            self.motor._connected = False
        finally:
            self._returning_to_zero = False
            self._re_enable_position_control()

    def _switch_loop(self):
        time.sleep(2.0)
        while self.running:
            self.switch_active = self.switch_sensor.is_active()
            time.sleep(0.02)

    def _deploy(self, z_target, accel, decel, max_rpm):
        self._deploying = True
        print(f"Deploying to {z_target}m at {max_rpm} RPM")
        try:
            target_turns = self.spool.get_motor_turns_for_length(z_target, self.switch_active)
            self.motor.set_traj_params(accel=accel, decel=decel, max_rpm=max_rpm)
            self.z_ref = z_target
            if self.motor.is_connected():
                self.motor.axis.controller.input_pos = target_turns
                timeout = time.time() + 30
                while time.time() < timeout:
                    pos = self.motor.get_motor_turns()
                    if abs(pos - target_turns) < 0.2:
                        break
                    time.sleep(0.05)
            print(f"Deployed to {z_target}m")
        except Exception as e:
            print(f"Deploy error: {e}")
        finally:
            self._deploying = False

    def _send_ff_position(self, z_target):
        """Feedforward pur — pas de correction"""
        ff_turns = self.spool.get_motor_turns_for_length(z_target, self.switch_active)
        if self.motor.is_connected():
            self.motor.axis.controller.input_pos = ff_turns

    def _tracking_loop(self):
        self._tracking = True
        print(f"Tracking started — center={self._tracking_center}m, amp={self._tracking_amplitude}m, freq={self._tracking_freq}Hz")
        self.z_ref = self._tracking_center
        self._send_ff_position(self._tracking_center)
        time.sleep(2.0)
        start_time = time.time()
        try:
            while self._tracking:
                t = time.time() - start_time
                z_ref = self._tracking_center + self._tracking_amplitude * math.sin(2 * math.pi * self._tracking_freq * t)
                self.z_ref = z_ref
                self._send_ff_position(z_ref)
                time.sleep(1 / TRACKING_FREQ)
        except Exception as e:
            print(f"Tracking error: {e}")
        finally:
            self._tracking = False
            print("Tracking stopped")

    def _prbs_loop(self, center, amplitude, freq_min, freq_max, seed=42):
        self._prbs_active = True
        random.seed(seed)
        print(f"PRBS started — center={center}m, amp={amplitude}m, freq=[{freq_min},{freq_max}]Hz")

        self.z_ref = center
        self._send_ff_position(center)
        time.sleep(2.0)

        state = 1
        try:
            while self._prbs_active:
                duration = random.uniform(1.0 / freq_max, 1.0 / freq_min)
                z_target = center + state * amplitude
                self.z_ref = z_target
                self._send_ff_position(z_target)
                print(f"PRBS: z={z_target:.2f}m, dur={duration:.2f}s")

                t_start = time.time()
                while time.time() - t_start < duration:
                    if not self._prbs_active:
                        break
                    time.sleep(0.02)

                state = -state
        except Exception as e:
            print(f"PRBS error: {e}")
        finally:
            self._prbs_active = False
            print("PRBS stopped")

    def _steps_loop(self, steps, accel, decel, max_rpm):
        self._steps_active = True
        self.motor.set_traj_params(accel=accel, decel=decel, max_rpm=max_rpm)
        print(f"Steps started: {len(steps)} steps")
        try:
            for i, step in enumerate(steps):
                if not self._steps_active:
                    break
                z = step['z']
                duration = step['duration']
                self._step_status = f"Step {i+1}/{len(steps)}: {z}m"
                print(self._step_status)
                self.z_ref = z
                self._send_ff_position(z)
                t_start = time.time()
                while time.time() - t_start < duration:
                    if not self._steps_active:
                        break
                    time.sleep(0.05)
            self._step_status = "Steps done"
            print("Steps done")
        except Exception as e:
            print(f"Steps error: {e}")
        finally:
            self._steps_active = False

    def _control_loop(self):
        while self.running:
            if self._calibrating or self._returning_to_zero or self._deploying:
                time.sleep(1 / CONTROL_FREQ)
                continue

            motor_turns = self.motor.get_motor_turns()
            motor_rpm = self.motor.get_rpm()
            current = self.motor.get_current()

            # Direction Hall depuis pos_estimate ODrive
            delta_dir = motor_turns - self._prev_motor_turns_dir
            if abs(delta_dir) > 0.01:
                if delta_dir > 0:
                    self.hall.set_direction(1)
                else:
                    self.hall.set_direction(-1)
                self._prev_motor_turns_dir = motor_turns

            delta = abs(motor_turns - self._prev_motor_turns)
            self._total_turns_motor += delta
            self._prev_motor_turns = motor_turns

            spool_rpm = self.hall.get_rpm()
            total_turns_spool = self.hall.get_total_turns()
            cable_length = total_turns_spool * self.spool.circumference

            if self._rpm_test_active or self._tracking or self._steps_active or self._prbs_active:
                self.logger.log(
                    motor_rpm, spool_rpm,
                    self._total_turns_motor, total_turns_spool,
                    cable_length, self.switch_active,
                    self.z_ref, current
                )
                time.sleep(1 / CONTROL_FREQ)
                continue

            if not self.switch_active and self.target_length > 0:
                target_motor_turns = self.spool.get_motor_turns_for_length(
                    self.target_length, self.switch_active
                )
                if self.motor.is_connected():
                    self.motor.set_target_turns(
                        target_motor_turns, self.slow_mode, self.custom_rpm
                    )
                self.z_ref = self.target_length

            if self.switch_active != self._prev_switch:
                self.logger.log_switch_event()
                self._prev_switch = self.switch_active

            self.logger.log(
                motor_rpm, spool_rpm,
                self._total_turns_motor, total_turns_spool,
                cable_length, self.switch_active,
                self.z_ref, current
            )

            if len(self.logger.timestamps) % (CONTROL_FREQ * 60) == 0 and len(self.logger.timestamps) > 0:
                self.logger.save_csv()

            time.sleep(1 / CONTROL_FREQ)

    def _run_calibration(self, conn):
        self._calibrating = True
        results = []
        self._send_message(conn, "Calibration started")

        if self.motor.is_connected():
            self.motor.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.motor.axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            self.motor.axis.controller.config.input_mode = InputMode.VEL_RAMP
            self.motor.axis.controller.config.vel_ramp_rate = 10

        for rpm in CALIBRATION_RPMS:
            self._send_message(conn, f"{rpm} RPM...")
            if self.motor.is_connected():
                self.motor.axis.controller.input_vel = rpm / 60
            time.sleep(CALIBRATION_WAIT)

            samples_motor = []
            samples_spool = []
            for _ in range(20):
                samples_motor.append(self.motor.get_rpm())
                samples_spool.append(self.hall.get_rpm())
                time.sleep(0.1)

            motor_rpm_avg = sum(samples_motor) / len(samples_motor)
            spool_rpm_avg = sum(samples_spool) / len(samples_spool)
            ratio = spool_rpm_avg / motor_rpm_avg if motor_rpm_avg > 1 else 0

            results.append({
                'target_rpm': rpm,
                'motor_rpm': round(motor_rpm_avg, 3),
                'spool_rpm': round(spool_rpm_avg, 3),
                'ratio': round(ratio, 4)
            })
            self._send_message(conn, f"{rpm} RPM done — Motor: {motor_rpm_avg:.1f} | Spool: {spool_rpm_avg:.1f} | Ratio: {ratio:.3f}")

        if self.motor.is_connected():
            self.motor.axis.controller.input_vel = 0

        path = f"calibration_{int(time.time())}.csv"
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['target_rpm', 'motor_rpm', 'spool_rpm', 'ratio'])
            writer.writeheader()
            writer.writerows(results)

        valid = [r['ratio'] for r in results if r['ratio'] > 0]
        avg_ratio = sum(valid) / len(valid) if valid else 0
        self._send_message(conn, f"Calibration done. Avg ratio: {avg_ratio:.3f} | CSV: {path}")

        self._calibrating = False
        self._re_enable_position_control()

    def _send_message(self, conn, msg):
        print(msg)
        try:
            snap = self.logger.get_snapshot()
            snap['calib_message'] = msg
            conn.sendall((json.dumps(snap) + '\n').encode())
        except:
            pass

    def _handle_client(self, conn):
        print("Client connected")
        conn.settimeout(0.1)
        buffer = ""
        while self.running:
            try:
                data = conn.recv(1024).decode()
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    cmd = json.loads(line)
                    self._process_command(cmd, conn)

                snapshot = self.logger.get_snapshot()
                snapshot['switch_active'] = self.switch_active
                snapshot['servo_pos'] = self.servo_pos
                snapshot['servo_min'] = self.servo_min
                snapshot['servo_max'] = self.servo_max
                snapshot['rpm_test_active'] = self._rpm_test_active
                snapshot['deploying'] = self._deploying
                snapshot['tracking'] = self._tracking
                snapshot['prbs_active'] = self._prbs_active
                snapshot['step_status'] = self._step_status
                conn.sendall((json.dumps(snapshot) + '\n').encode())

            except socket.timeout:
                try:
                    snapshot = self.logger.get_snapshot()
                    snapshot['switch_active'] = self.switch_active
                    snapshot['servo_pos'] = self.servo_pos
                    snapshot['servo_min'] = self.servo_min
                    snapshot['servo_max'] = self.servo_max
                    snapshot['rpm_test_active'] = self._rpm_test_active
                    snapshot['deploying'] = self._deploying
                    snapshot['tracking'] = self._tracking
                    snapshot['prbs_active'] = self._prbs_active
                    snapshot['step_status'] = self._step_status
                    conn.sendall((json.dumps(snapshot) + '\n').encode())
                except:
                    break

            except Exception as e:
                print(f"Client error: {e}")
                break
        print("Client disconnected")

    def _process_command(self, cmd, conn=None):
        if 'target_length' in cmd:
            self.target_length = float(cmd['target_length'])
            self.z_ref = self.target_length
            self._total_turns_motor = 0.0
            self._prev_motor_turns = self.motor.get_motor_turns()
        if 'slow_mode' in cmd:
            self.slow_mode = bool(cmd['slow_mode'])
        if 'custom_rpm' in cmd:
            self.custom_rpm = float(cmd['custom_rpm']) if cmd['custom_rpm'] else None
        if 'save_csv' in cmd:
            self.logger.save_csv()
        if 'estop' in cmd:
            self.motor.disable()
            self.target_length = 0.0
            self._returning_to_zero = False
            self._rpm_test_active = False
            self._deploying = False
            self._tracking = False
            self._prbs_active = False
            self._steps_active = False
            print("E-STOP triggered")
        if 'calibrate' in cmd and conn:
            threading.Thread(target=self._run_calibration, args=(conn,), daemon=True).start()
        if 'reset' in cmd:
            self.logger.reset()
            self.hall.reset()
            self.spool.reset()
            self._total_turns_motor = 0.0
            self.z_ref = 0.0
            self.target_length = 0.0
            self._step_status = ""
            self._prev_motor_turns_dir = 0.0
            if self.motor.is_connected():
                try:
                    self.motor.axis.set_abs_pos(0)
                    self._prev_motor_turns = 0.0
                except Exception as e:
                    print(f"Reset position error: {e}")
        if 'servo_pos' in cmd:
            self.servo_pos = float(cmd['servo_pos'])
            self.servo.set_position(self.servo_pos)
        if 'servo_min' in cmd:
            self.servo_min = self.servo_pos
        if 'servo_max' in cmd:
            self.servo_max = self.servo_pos
        if 'servo_mid' in cmd:
            self.servo.mid()
            self.servo_pos = 0.0
        if 'servo_lock' in cmd:
            self.servo_pos = self.servo_min
            self.servo.set_position(self.servo_min)
        if 'servo_unlock' in cmd:
            self.servo_pos = self.servo_max
            self.servo.set_position(self.servo_max)
        if 'rpm_test_start' in cmd:
            self._rpm_test_value = float(cmd['rpm_test_start'])
            self._rpm_test_active = True
            if self.motor.is_connected():
                try:
                    self.motor.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    self.motor.axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
                    self.motor.axis.controller.config.input_mode = InputMode.VEL_RAMP
                    self.motor.axis.controller.config.vel_ramp_rate = 10
                    self.motor.axis.controller.input_vel = self._rpm_test_value / 60
                except Exception as e:
                    print(f"RPM test start error: {e}")
        if 'rpm_test_stop' in cmd:
            self._rpm_test_active = False
            if self.motor.is_connected():
                try:
                    self.motor.axis.controller.input_vel = 0
                except Exception as e:
                    print(f"RPM test stop error: {e}")
            self._re_enable_position_control()
        if 'deploy' in cmd:
            z = float(cmd['deploy']['z'])
            accel = float(cmd['deploy'].get('accel', 5.0))
            decel = float(cmd['deploy'].get('decel', 3.0))
            max_rpm = float(cmd['deploy'].get('max_rpm', 1000))
            threading.Thread(
                target=self._deploy,
                args=(z, accel, decel, max_rpm),
                daemon=True
            ).start()
        if 'set_traj_params' in cmd:
            p = cmd['set_traj_params']
            self.motor.set_traj_params(
                accel=p.get('accel'),
                decel=p.get('decel'),
                max_rpm=p.get('max_rpm')
            )
        if 'start_tracking' in cmd:
            p = cmd['start_tracking']
            self._tracking_center = float(p.get('center', 2.0))
            self._tracking_amplitude = float(p.get('amplitude', 0.5))
            self._tracking_freq = float(p.get('freq', 0.5))
            self._tracking = True
            threading.Thread(target=self._tracking_loop, daemon=True).start()
        if 'stop_tracking' in cmd:
            self._tracking = False
        if 'start_prbs' in cmd:
            p = cmd['start_prbs']
            threading.Thread(
                target=self._prbs_loop,
                args=(
                    float(p.get('center', 2.0)),
                    float(p.get('amplitude', 0.5)),
                    float(p.get('freq_min', 0.5)),
                    float(p.get('freq_max', 2.0)),
                    int(p.get('seed', 42))
                ),
                daemon=True
            ).start()
        if 'stop_prbs' in cmd:
            self._prbs_active = False
        if 'start_steps' in cmd:
            p = cmd['start_steps']
            threading.Thread(
                target=self._steps_loop,
                args=(p['steps'], p.get('accel', 5.0), p.get('decel', 3.0), p.get('max_rpm', 1000)),
                daemon=True
            ).start()
        if 'stop_steps' in cmd:
            self._steps_active = False

    def start(self):
        os.system("sudo fuser -k 5005/tcp 2>/dev/null")
        time.sleep(0.5)

        self.connect_motor()
        self.running = True

        switch_thread = threading.Thread(target=self._switch_loop, daemon=True)
        switch_thread.start()

        ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        ctrl_thread.start()

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((HOST, PORT))
        sock.listen(1)
        print(f"Server listening on {HOST}:{PORT}")

        while self.running:
            conn, addr = sock.accept()
            client_thread = threading.Thread(
                target=self._handle_client, args=(conn,), daemon=True
            )
            client_thread.start()

if __name__ == "__main__":
    server = WASPServer()
    server.start()