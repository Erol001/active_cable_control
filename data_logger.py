# data_logger.py

import time
import csv
import os

class DataLogger:
    def __init__(self):
        self.reset()

    def reset(self):
        self.timestamps = []
        self.motor_rpms = []
        self.spool_rpms = []
        self.total_turns_motor = []
        self.total_turns_spool = []
        self.cable_lengths = []
        self.switch_states = []
        self.z_refs = []
        self.currents = []
        self.switch_events = []
        self._start_time = time.time()

    def log(self, motor_rpm, spool_rpm, total_turns_motor, total_turns_spool,
            cable_length, switch_active, z_ref=0.0, current=0.0):
        t = time.time() - self._start_time
        self.timestamps.append(t)
        self.motor_rpms.append(motor_rpm)
        self.spool_rpms.append(spool_rpm)
        self.total_turns_motor.append(total_turns_motor)
        self.total_turns_spool.append(total_turns_spool)
        self.cable_lengths.append(cable_length)
        self.switch_states.append(switch_active)
        self.z_refs.append(z_ref)
        self.currents.append(current)

    def log_switch_event(self):
        t = time.time() - self._start_time
        self.switch_events.append(t)

    def get_snapshot(self):
        return {
            "t": self.timestamps[-1] if self.timestamps else 0,
            "motor_rpm": self.motor_rpms[-1] if self.motor_rpms else 0,
            "spool_rpm": self.spool_rpms[-1] if self.spool_rpms else 0,
            "total_turns_motor": self.total_turns_motor[-1] if self.total_turns_motor else 0,
            "total_turns_spool": self.total_turns_spool[-1] if self.total_turns_spool else 0,
            "cable_length": self.cable_lengths[-1] if self.cable_lengths else 0,
            "switch_active": self.switch_states[-1] if self.switch_states else False,
            "z_ref": self.z_refs[-1] if self.z_refs else 0,
            "current": self.currents[-1] if self.currents else 0,
            "switch_events": self.switch_events,
        }

    def save_csv(self, path=None):
        if path is None:
            path = f"wasp_log_{int(time.time())}.csv"
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'motor_rpm', 'spool_rpm',
                'total_turns_motor', 'total_turns_spool',
                'cable_length', 'switch_active', 'z_ref', 'current'
            ])
            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i],
                    self.motor_rpms[i],
                    self.spool_rpms[i],
                    self.total_turns_motor[i],
                    self.total_turns_spool[i],
                    self.cable_lengths[i],
                    self.switch_states[i],
                    self.z_refs[i],
                    self.currents[i],
                ])
        print(f"CSV saved: {path}")
        return path