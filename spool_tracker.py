# spool_tracker.py

import math
from config import SPOOL_DIAMETER, GEAR_RATIO_SLOW, GEAR_RATIO_FAST

class SpoolTracker:
    def __init__(self):
        self.spool_turns = 0.0
        self.cable_length = 0.0
        self.circumference = math.pi * SPOOL_DIAMETER

    def update(self, motor_turns, switch_active):
        gear_ratio = GEAR_RATIO_SLOW if switch_active else GEAR_RATIO_FAST
        self.spool_turns = motor_turns * gear_ratio
        self.cable_length = self.spool_turns * self.circumference
        return self.spool_turns, self.cable_length

    def get_motor_turns_for_length(self, target_length, switch_active):
        gear_ratio = GEAR_RATIO_SLOW if switch_active else GEAR_RATIO_FAST
        target_spool_turns = target_length / self.circumference
        target_motor_turns = target_spool_turns / gear_ratio
        return -target_motor_turns

    def reset(self):
        self.spool_turns = 0.0
        self.cable_length = 0.0