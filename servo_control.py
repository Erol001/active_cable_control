# servo_control.py

import lgpio
import time
import threading
from config import SERVO_PIN

class ServoController:
    def __init__(self):
        self.h = lgpio.gpiochip_open(0)
        self.pin = SERVO_PIN
        self._pos = 0.0
        lgpio.gpio_claim_output(self.h, self.pin)
        print("Servo initialized with lgpio")

    def _pulse_width(self, value):
        # value -1 to 1 → 500µs to 2500µs
        return int(500 + (value + 1) * 1000)

    def set_position(self, value):
        value = max(-1, min(1, value))
        self._pos = value
        pw = self._pulse_width(value)
        lgpio.tx_servo(self.h, self.pin, pw)

    def mid(self):
        self.set_position(0)

    def detach(self):
        lgpio.tx_servo(self.h, self.pin, 0)