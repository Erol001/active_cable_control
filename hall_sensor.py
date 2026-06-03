# hall_sensor.py

import threading
import time
from gpiozero import Button
from config import HALL_PIN, N_MAGNETS, BOUNCE_TIME

class HallSensor:
    def __init__(self):
        self.sensor = Button(HALL_PIN, pull_up=False, bounce_time=BOUNCE_TIME)
        self._lock = threading.Lock()
        self._pulse_times = []
        self._pulse_log = []
        self.total_turns = 0.0
        self._direction = 1
        self.sensor.when_pressed = self._on_pulse

    def _on_pulse(self):
        now = time.time()
        with self._lock:
            self._pulse_times.append(now)
            if len(self._pulse_times) > 20:
                self._pulse_times.pop(0)
            self.total_turns += self._direction / N_MAGNETS
            self._pulse_log.append((now, self._direction, self.total_turns))

    def set_direction(self, direction):
        with self._lock:
            self._direction = direction

    def get_rpm(self):
        with self._lock:
            now = time.time()
            recent = [t for t in self._pulse_times if now - t < 2.0]
            if len(recent) < 2:
                return 0.0
            intervals = [recent[i+1] - recent[i] for i in range(len(recent)-1)]
            avg_interval = sum(intervals) / len(intervals)
            return (60 / avg_interval) / N_MAGNETS if avg_interval > 0 else 0.0

    def get_total_turns(self):
        with self._lock:
            return self.total_turns

    def get_pulse_log(self):
        with self._lock:
            return list(self._pulse_log)

    def reset(self):
        with self._lock:
            self._pulse_times = []
            self._pulse_log = []
            self.total_turns = 0.0
            self._direction = 1