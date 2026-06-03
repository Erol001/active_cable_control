# switch_sensor.py

from gpiozero import Button
from config import SWITCH_PIN

class SwitchSensor:
    def __init__(self):
        self.sensor = Button(SWITCH_PIN, pull_up=True)

    def is_active(self):
        return self.sensor.is_pressed