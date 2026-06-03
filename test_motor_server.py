# test_motor_server.py

import socket
import json
import threading
import time
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from gpiozero import Button

HOST = '0.0.0.0'
PORT = 5006
HALL_PIN = 6

pulse_times = []
total_turns = 0.0
lock = threading.Lock()

sensor = Button(HALL_PIN, pull_up=False, bounce_time=0.001)

def on_pulse():
    global total_turns
    now = time.time()
    with lock:
        pulse_times.append(now)
        if len(pulse_times) > 10:
            pulse_times.pop(0)
        total_turns += 1

sensor.when_pressed = on_pulse

def get_rpm():
    with lock:
        now = time.time()
        # Filtre les pulses des 2 dernières secondes
        recent = [t for t in pulse_times if now - t < 2.0]
        if len(recent) < 2:
            return 0.0
        # RPM moyen sur les derniers pulses
        intervals = [recent[i+1] - recent[i] for i in range(len(recent)-1)]
        avg_interval = sum(intervals) / len(intervals)
        return 60 / avg_interval if avg_interval > 0 else 0.0

def print_hall():
    while True:
        time.sleep(0.5)
        rpm = get_rpm()
        with lock:
            turns = total_turns
        print(f"Spool RPM: {rpm:.1f} | Total turns: {turns:.1f}")

def main():
    print("Connecting to ODrive...")
    odrv = odrive.find_any()
    axis = odrv.axis0
    print(f"Connected. Vbus: {odrv.vbus_voltage:.2f}V")
    axis.controller.config.vel_ramp_rate = 10

    threading.Thread(target=print_hall, daemon=True).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen(1)
    print(f"Waiting for connection on port {PORT}...")

    conn, addr = sock.accept()
    conn.settimeout(5.0)
    print(f"Connected: {addr}")

    buffer = ""
    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                cmd = json.loads(line)

                if cmd.get('action') == 'enable':
                    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
                    axis.controller.config.input_mode = InputMode.VEL_RAMP
                    print("Motor enabled")

                elif cmd.get('action') == 'set_rpm':
                    rpm = float(cmd['rpm'])
                    axis.controller.input_vel = rpm / 60
                    print(f"RPM set to {rpm}")

                elif cmd.get('action') == 'stop':
                    axis.controller.input_vel = 0
                    axis.requested_state = AxisState.IDLE
                    print("Motor stopped")

            telem = {
                'vbus': odrv.vbus_voltage,
                'vel': axis.vel_estimate * 60,
                'current': axis.motor.foc.Iq_measured,
            }
            conn.sendall((json.dumps(telem) + '\n').encode())

        except socket.timeout:
            try:
                telem = {
                    'vbus': odrv.vbus_voltage,
                    'vel': axis.vel_estimate * 60,
                    'current': axis.motor.foc.Iq_measured,
                }
                conn.sendall((json.dumps(telem) + '\n').encode())
            except:
                break

        except Exception as e:
            print(f"Error: {e}")
            break

    axis.controller.input_vel = 0
    axis.requested_state = AxisState.IDLE
    conn.close()
    print("Disconnected safely")

if __name__ == "__main__":
    main()