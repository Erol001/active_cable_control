# gui_control.py

import tkinter as tk
from tkinter import ttk
import socket
import json
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

HOST = 'raspberrypi.local'
PORT = 5005
MAX_POINTS = 200

class WASPGui:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.buffer = ""
        self._lock = threading.Lock()

        self.times = deque(maxlen=MAX_POINTS)
        self.motor_rpms = deque(maxlen=MAX_POINTS)
        self.spool_rpms = deque(maxlen=MAX_POINTS)
        self.total_turns_motor = deque(maxlen=MAX_POINTS)
        self.total_turns_spool = deque(maxlen=MAX_POINTS)
        self.cable_lengths = deque(maxlen=MAX_POINTS)
        self.currents = deque(maxlen=MAX_POINTS)
        self.switch_events = []
        self._was_deploying = False

        self.rpm_test_times = deque(maxlen=MAX_POINTS)
        self.rpm_test_spool = deque(maxlen=MAX_POINTS)
        self.rpm_test_motor = deque(maxlen=MAX_POINTS)

        self._build_gui()
        self._build_plots()
        self._build_rpm_test_plot()

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("WASP Control")

        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(row=0, column=0, padx=5, pady=5)
        self.conn_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.conn_label.grid(row=0, column=1, padx=5)

        target_frame = ttk.LabelFrame(self.root, text="Target")
        target_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(target_frame, text="Target length z (m):").grid(row=0, column=0, sticky="w")
        self.target_length = ttk.Entry(target_frame, width=10)
        self.target_length.insert(0, "1.0")
        self.target_length.grid(row=0, column=1, padx=5)
        ttk.Button(target_frame, text="Send", command=self.send_target).grid(row=0, column=2, padx=5)

        rpm_frame = ttk.LabelFrame(self.root, text="Motor RPM")
        rpm_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(rpm_frame, text="Max RPM:").grid(row=0, column=0, sticky="w")
        self.custom_rpm = ttk.Entry(rpm_frame, width=10)
        self.custom_rpm.insert(0, "500")
        self.custom_rpm.grid(row=0, column=1, padx=5)
        ttk.Button(rpm_frame, text="Set", command=self.send_rpm).grid(row=0, column=2, padx=5)

        mode_frame = ttk.LabelFrame(self.root, text="Mode")
        mode_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.slow_mode = tk.BooleanVar(value=True)
        ttk.Radiobutton(mode_frame, text="Slow", variable=self.slow_mode, value=True,
                        command=self.send_mode).grid(row=0, column=0, padx=5)
        ttk.Radiobutton(mode_frame, text="Fast", variable=self.slow_mode, value=False,
                        command=self.send_mode).grid(row=0, column=1, padx=5)

        # ── Torque mode ───────────────────────────────────────────
        torque_frame = ttk.LabelFrame(self.root, text="Torque Mode")
        torque_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(torque_frame, text="Torque (Nm):").grid(row=0, column=0, sticky="w", padx=5)
        self.torque_entry = ttk.Entry(torque_frame, width=8)
        self.torque_entry.insert(0, "0.05")
        self.torque_entry.grid(row=0, column=1, padx=5)

        ttk.Label(torque_frame, text="Vel limit (RPM):").grid(row=1, column=0, sticky="w", padx=5)
        self.torque_vel_limit = ttk.Entry(torque_frame, width=8)
        self.torque_vel_limit.insert(0, "900")
        self.torque_vel_limit.grid(row=1, column=1, padx=5)

        btn_t = ttk.Frame(torque_frame)
        btn_t.grid(row=2, column=0, columnspan=2, pady=5)
        ttk.Button(btn_t, text="▶  Start torque", command=self.send_torque_start).grid(row=0, column=0, padx=4)
        ttk.Button(btn_t, text="■  Stop",         command=self.send_torque_stop).grid(row=0, column=1, padx=4)

        self.torque_status = ttk.Label(torque_frame, text="Torque: OFF", foreground="gray")
        self.torque_status.grid(row=3, column=0, columnspan=2, padx=5, pady=2)

        switch_frame = ttk.LabelFrame(self.root, text="Switch")
        switch_frame.grid(row=5, column=0, padx=10, pady=5, sticky="ew")
        self.switch_label = ttk.Label(switch_frame, text="Switch: OFF",
                                       foreground="red", font=("Arial", 11, "bold"))
        self.switch_label.grid(row=0, column=0, padx=10, pady=5)

        servo_frame = ttk.LabelFrame(self.root, text="Servo")
        servo_frame.grid(row=6, column=0, padx=10, pady=5, sticky="ew")

        self.servo_slider = ttk.Scale(servo_frame, from_=-1.0, to=1.0,
                                       orient='horizontal', length=200,
                                       command=self._on_servo_slide)
        self.servo_slider.set(0.0)
        self.servo_slider.grid(row=0, column=0, columnspan=4, padx=5, pady=5)

        self.servo_val_label = ttk.Label(servo_frame, text="Pos: 0.00")
        self.servo_val_label.grid(row=0, column=4, padx=5)

        ttk.Button(servo_frame, text="Mid",     command=self.send_servo_mid).grid(row=1, column=0, padx=5, pady=3)
        ttk.Button(servo_frame, text="Set Min", command=self.send_servo_set_min).grid(row=1, column=1, padx=5)
        ttk.Button(servo_frame, text="Set Max", command=self.send_servo_set_max).grid(row=1, column=2, padx=5)

        self.servo_range_label = ttk.Label(servo_frame, text="Range: -- / --")
        self.servo_range_label.grid(row=2, column=0, columnspan=5, padx=5)

        tk.Button(servo_frame, text="LOCK", command=self.send_servo_lock,
                  bg="navy", fg="white", font=("Arial", 10, "bold")).grid(row=3, column=0, padx=5, pady=5)
        tk.Button(servo_frame, text="UNLOCK", command=self.send_servo_unlock,
                  bg="darkgreen", fg="white", font=("Arial", 10, "bold")).grid(row=3, column=1, padx=5, pady=5)

        rpm_test_frame = ttk.LabelFrame(self.root, text="RPM Test")
        rpm_test_frame.grid(row=7, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(rpm_test_frame, text="Motor RPM:").grid(row=0, column=0, sticky="w", padx=5)
        self.rpm_test_entry = ttk.Entry(rpm_test_frame, width=10)
        self.rpm_test_entry.insert(0, "100")
        self.rpm_test_entry.grid(row=0, column=1, padx=5)
        ttk.Button(rpm_test_frame, text="Start", command=self.send_rpm_test_start).grid(row=0, column=2, padx=5)
        ttk.Button(rpm_test_frame, text="Stop",  command=self.send_rpm_test_stop).grid(row=0, column=3, padx=5)
        self.rpm_test_label = ttk.Label(rpm_test_frame, text="Spool RPM: --")
        self.rpm_test_label.grid(row=1, column=0, columnspan=4, padx=5)

        config_frame = ttk.LabelFrame(self.root, text="Config")
        config_frame.grid(row=8, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(config_frame, text="Spool diameter (m):").grid(row=0, column=0, sticky="w")
        self.spool_diameter = ttk.Entry(config_frame, width=10)
        self.spool_diameter.insert(0, "0.05")
        self.spool_diameter.grid(row=0, column=1, padx=5)

        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=9, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(btn_frame, text="Reset",     command=self.send_reset).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Save CSV",  command=self.send_save_csv).grid(row=0, column=1, padx=5)
        ttk.Button(btn_frame, text="Calibrate", command=self.send_calibrate).grid(row=0, column=2, padx=5)

        estop_btn = tk.Button(self.root, text="E-STOP", command=self.send_estop,
                              bg="red", fg="white", font=("Arial", 14, "bold"), height=2)
        estop_btn.grid(row=10, column=0, padx=10, pady=10, sticky="ew")

        calib_frame = ttk.LabelFrame(self.root, text="Calibration")
        calib_frame.grid(row=11, column=0, padx=10, pady=5, sticky="ew")
        self.calib_text = tk.Text(calib_frame, height=6, width=50, state='disabled',
                                   bg='black', fg='lime', font=('Courier', 9))
        self.calib_text.grid(row=0, column=0, padx=5, pady=5)

        telem_frame = ttk.LabelFrame(self.root, text="Telemetry")
        telem_frame.grid(row=12, column=0, padx=10, pady=5, sticky="ew")
        self.motor_rpm_label = ttk.Label(telem_frame, text="Motor RPM: --")
        self.motor_rpm_label.grid(row=0, column=0, padx=5)
        self.spool_rpm_label = ttk.Label(telem_frame, text="Spool RPM: --")
        self.spool_rpm_label.grid(row=0, column=1, padx=5)
        self.length_label = ttk.Label(telem_frame, text="Cable: -- m")
        self.length_label.grid(row=0, column=2, padx=5)
        self.current_telem_label = ttk.Label(telem_frame, text="Current: -- A")
        self.current_telem_label.grid(row=0, column=3, padx=5)

    def _build_plots(self):
        self.fig, axes = plt.subplots(6, 1, figsize=(8, 14))
        self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6 = axes
        self.fig.suptitle("WASP Telemetry")
        self.ani = animation.FuncAnimation(
            self.fig, self._update_plots, interval=200, cache_frame_data=False
        )

    def _build_rpm_test_plot(self):
        self.fig2, self.ax_rpm = plt.subplots(1, 1, figsize=(8, 4))
        self.fig2.suptitle("RPM Test — Live")
        self.ani2 = animation.FuncAnimation(
            self.fig2, self._update_rpm_test_plot, interval=200, cache_frame_data=False
        )

    def _update_plots(self, frame):
        with self._lock:
            t = list(self.times)
            motor_rpms = list(self.motor_rpms)
            spool_rpms = list(self.spool_rpms)
            total_turns_motor = list(self.total_turns_motor)
            total_turns_spool = list(self.total_turns_spool)
            cable_lengths = list(self.cable_lengths)
            currents = list(self.currents)
            switch_events = list(self.switch_events)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.clear()

        self.ax1.plot(t, motor_rpms, 'b-')
        self.ax1.set_ylabel("Motor RPM")

        self.ax2.scatter(t, spool_rpms, color='green', s=2)
        self.ax2.set_ylabel("Spool RPM")

        self.ax3.plot(t, total_turns_motor, 'b--')
        self.ax3.set_ylabel("Total turns motor")

        self.ax4.plot(t, total_turns_spool, 'g--')
        self.ax4.set_ylabel("Total turns spool")

        self.ax5.plot(t, cable_lengths, 'orange')
        self.ax5.set_ylabel("Cable length (m)")

        self.ax6.plot(t, currents, color='orange')
        self.ax6.set_ylabel("Current (A)")
        self.ax6.set_xlabel("Time (s)")

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.tick_params(labelbottom=True)
            for t_ev in switch_events:
                ax.axvline(x=t_ev, color='red', linewidth=1.5, linestyle='--')

    def _update_rpm_test_plot(self, frame):
        with self._lock:
            t = list(self.rpm_test_times)
            spool = list(self.rpm_test_spool)
            motor = list(self.rpm_test_motor)

        self.ax_rpm.clear()
        self.ax_rpm.plot(t, motor, 'b-', label='Motor RPM')
        self.ax_rpm.scatter(t, spool, color='green', s=3, label='Spool RPM')
        self.ax_rpm.set_ylabel("RPM")
        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.legend()
        self.ax_rpm.grid(True, alpha=0.3)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
            self.connected = True
            self.conn_label.config(text="Connected", foreground="white")
            threading.Thread(target=self._receive_loop, daemon=True).start()
        except Exception as e:
            self.conn_label.config(text=f"Error: {e}", foreground="red")

    def _receive_loop(self):
        while self.connected:
            try:
                data = self.sock.recv(1024).decode()
                if not data:
                    break
                self.buffer += data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    snap = json.loads(line)
                    self._update_telemetry(snap)
            except Exception as e:
                print(f"Receive error: {e}")
                break

    def _update_telemetry(self, snap):
        with self._lock:
            self.times.append(snap['t'])
            self.motor_rpms.append(snap['motor_rpm'])
            self.spool_rpms.append(snap['spool_rpm'])
            self.total_turns_motor.append(snap['total_turns_motor'])
            self.total_turns_spool.append(snap['total_turns_spool'])
            self.cable_lengths.append(snap['cable_length'])
            self.currents.append(snap.get('current', 0))
            self.switch_events = snap['switch_events']

            if snap.get('rpm_test_active', False):
                self.rpm_test_times.append(snap['t'])
                self.rpm_test_spool.append(snap['spool_rpm'])
                self.rpm_test_motor.append(snap['motor_rpm'])

            deploying = snap.get('deploying', False)
            if self._was_deploying and not deploying:
                self.root.after(0, lambda: self._send({'servo_unlock': True}))
            self._was_deploying = deploying

        self.motor_rpm_label.config(text=f"Motor RPM: {snap['motor_rpm']:.1f}")
        self.spool_rpm_label.config(text=f"Spool RPM: {snap['spool_rpm']:.1f}")
        self.length_label.config(text=f"Cable: {snap['cable_length']:.3f} m")
        self.current_telem_label.config(text=f"Current: {snap.get('current', 0):.2f} A")

        if snap.get('rpm_test_active', False):
            self.rpm_test_label.config(text=f"Spool RPM: {snap['spool_rpm']:.1f}")

        if snap.get('switch_active'):
            self.switch_label.config(text="Switch: ON (avion en bas)", foreground="green")
        else:
            self.switch_label.config(text="Switch: OFF (vol libre)", foreground="red")

        if 'servo_min' in snap and 'servo_max' in snap:
            self.servo_range_label.config(
                text=f"Range: {snap['servo_min']:.2f} / {snap['servo_max']:.2f}"
            )

        if snap.get('torque_active', False):
            self.torque_status.config(
                text=f"Torque: ON — {snap.get('torque_setpoint', 0):.3f} Nm",
                foreground="green")
        else:
            self.torque_status.config(text="Torque: OFF", foreground="gray")

        if 'calib_message' in snap:
            self._append_calib_message(snap['calib_message'])

    def _append_calib_message(self, msg):
        self.calib_text.config(state='normal')
        self.calib_text.insert(tk.END, f"> {msg}\n")
        self.calib_text.see(tk.END)
        self.calib_text.config(state='disabled')

    def _on_servo_slide(self, val):
        pos = float(val)
        self.servo_val_label.config(text=f"Pos: {pos:.2f}")
        self._send({'servo_pos': pos})

    def _send(self, cmd):
        if self.connected:
            try:
                self.sock.sendall((json.dumps(cmd) + '\n').encode())
            except Exception as e:
                print(f"Send error: {e}")
                self.connected = False
                self.conn_label.config(text="Disconnected", foreground="red")

    def send_target(self):
        self._send({'target_length': float(self.target_length.get())})

    def send_rpm(self):
        self._send({'custom_rpm': float(self.custom_rpm.get())})

    def send_mode(self):
        self._send({'slow_mode': self.slow_mode.get()})

    def send_torque_start(self):
        self._send({
            'torque_start': {
                'torque':     float(self.torque_entry.get()),
                'vel_limit':  float(self.torque_vel_limit.get()) / 60,
            }
        })

    def send_torque_stop(self):
        self._send({'torque_stop': True})

    def send_servo_mid(self):
        self.servo_slider.set(0.0)
        self._send({'servo_mid': True})

    def send_servo_set_min(self):
        self._send({'servo_min': True})

    def send_servo_set_max(self):
        self._send({'servo_max': True})

    def send_servo_lock(self):
        self._send({'servo_lock': True})

    def send_servo_unlock(self):
        self._send({'servo_unlock': True})

    def send_rpm_test_start(self):
        rpm = float(self.rpm_test_entry.get())
        with self._lock:
            self.rpm_test_times.clear()
            self.rpm_test_spool.clear()
            self.rpm_test_motor.clear()
        self._send({'rpm_test_start': rpm})

    def send_rpm_test_stop(self):
        self._send({'rpm_test_stop': True})

    def send_reset(self):
        self._send({'reset': True})
        with self._lock:
            self.times.clear()
            self.motor_rpms.clear()
            self.spool_rpms.clear()
            self.total_turns_motor.clear()
            self.total_turns_spool.clear()
            self.cable_lengths.clear()
            self.currents.clear()
            self.switch_events.clear()

    def send_save_csv(self):  self._send({'save_csv': True})
    def send_estop(self):     self._send({'estop': True})

    def send_calibrate(self):
        self._append_calib_message("Sending calibration command...")
        self._send({'calibrate': True})

    def run(self):
        plt.show(block=False)
        self.root.mainloop()

if __name__ == "__main__":
    app = WASPGui()
    app.run()