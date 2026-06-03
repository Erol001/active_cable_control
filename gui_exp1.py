# gui_exp1.py

import tkinter as tk
from tkinter import ttk
import socket
import json
import threading
import matplotlib.pyplot as plt
from collections import deque

HOST = 'raspberrypi.local'
PORT = 5005
MAX_POINTS = 500

class GUIExp1:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.buffer = ""
        self._lock = threading.Lock()

        self.times = deque(maxlen=MAX_POINTS)
        self.cable_lengths = deque(maxlen=MAX_POINTS)
        self.z_refs = deque(maxlen=MAX_POINTS)
        self.motor_rpms = deque(maxlen=MAX_POINTS)
        self.spool_rpms = deque(maxlen=MAX_POINTS)
        self.currents = deque(maxlen=MAX_POINTS)
        self.motor_accels = deque(maxlen=MAX_POINTS)
        self.switch_events = []
        self._was_deploying = False

        self._build_gui()
        self._build_plots()

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Exp 1 — Déploiement avec charge")

        # Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(row=0, column=0, padx=5, pady=5)
        self.conn_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.conn_label.grid(row=0, column=1, padx=5)

        # Deploy
        deploy_frame = ttk.LabelFrame(self.root, text="Déploiement")
        deploy_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        # Direction
        ttk.Label(deploy_frame, text="Direction:").grid(row=0, column=0, sticky="w", padx=5, pady=(4,0))
        self.direction = tk.StringVar(value="deploy")
        ttk.Radiobutton(deploy_frame, text="Déployer ↓ (câble sort)",
                        variable=self.direction, value="deploy").grid(
            row=0, column=1, columnspan=2, sticky="w")
        ttk.Radiobutton(deploy_frame, text="Rétracter ↑ (câble rentre)",
                        variable=self.direction, value="retract").grid(
            row=1, column=1, columnspan=2, sticky="w")

        ttk.Separator(deploy_frame, orient="horizontal").grid(
            row=2, column=0, columnspan=3, sticky="ew", pady=4)

        ttk.Label(deploy_frame, text="z cible (m):").grid(row=3, column=0, sticky="w", padx=5)
        self.z_target = ttk.Entry(deploy_frame, width=8)
        self.z_target.insert(0, "4.0")
        self.z_target.grid(row=3, column=1, padx=5)

        ttk.Label(deploy_frame, text="Max RPM:").grid(row=4, column=0, sticky="w", padx=5)
        self.max_rpm = ttk.Entry(deploy_frame, width=8)
        self.max_rpm.insert(0, "1000")
        self.max_rpm.grid(row=4, column=1, padx=5)

        # Ramped mode
        self.ramped_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(deploy_frame, text="Ramped mode",
                        variable=self.ramped_var,
                        command=self._toggle_ramp).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=5, pady=(6, 0))

        self.lbl_accel = ttk.Label(deploy_frame, text="Accel (tr/s²):")
        self.lbl_decel = ttk.Label(deploy_frame, text="Decel (tr/s²):")
        self.accel = ttk.Entry(deploy_frame, width=8); self.accel.insert(0, "5.0")
        self.decel = ttk.Entry(deploy_frame, width=8); self.decel.insert(0, "3.0")

        self.lbl_accel.grid(row=6, column=0, sticky="w", padx=5)
        self.accel.grid(row=6, column=1, padx=5)
        self.lbl_decel.grid(row=7, column=0, sticky="w", padx=5)
        self.decel.grid(row=7, column=1, padx=5)

        self.auto_servo_lock = tk.BooleanVar(value=True)
        ttk.Checkbutton(deploy_frame, text="Servo lock auto après déploiement",
                        variable=self.auto_servo_lock).grid(
            row=8, column=0, columnspan=3, sticky="w", padx=5, pady=(4, 0))

        ttk.Button(deploy_frame, text="▶  DEPLOY", command=self.send_deploy).grid(
            row=9, column=0, columnspan=3, pady=8, ipadx=10)

        # Switch
        switch_frame = ttk.LabelFrame(self.root, text="Switch")
        switch_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        self.switch_label = ttk.Label(switch_frame, text="Switch: OFF",
                                      foreground="red", font=("Arial", 11, "bold"))
        self.switch_label.grid(row=0, column=0, padx=10, pady=5)
        self.switch_time_label = ttk.Label(switch_frame, text="Décommutation: --")
        self.switch_time_label.grid(row=1, column=0, padx=10)

        # Telemetry
        telem_frame = ttk.LabelFrame(self.root, text="Telemetry")
        telem_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.motor_rpm_label = ttk.Label(telem_frame, text="Motor RPM: --")
        self.motor_rpm_label.grid(row=0, column=0, padx=5)
        self.spool_rpm_label = ttk.Label(telem_frame, text="Spool RPM: --")
        self.spool_rpm_label.grid(row=0, column=1, padx=5)
        self.length_label = ttk.Label(telem_frame, text="Cable: -- m")
        self.length_label.grid(row=0, column=2, padx=5)
        self.current_label = ttk.Label(telem_frame, text="Current: -- A")
        self.current_label.grid(row=0, column=3, padx=5)

        # Buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(btn_frame, text="Reset", command=self.send_reset).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Save CSV", command=self.send_save_csv).grid(row=0, column=1, padx=5)

        tk.Button(self.root, text="⚡  E-STOP", command=self.send_estop,
                  bg="red", fg="white", font=("Arial", 14, "bold"), height=2).grid(
            row=5, column=0, padx=10, pady=10, sticky="ew")

    def _toggle_ramp(self):
        state = "normal" if self.ramped_var.get() else "disabled"
        self.accel.config(state=state)
        self.decel.config(state=state)

    def _build_plots(self):
        self.fig, axes = plt.subplots(5, 1, figsize=(9, 13))
        self.ax1, self.ax2, self.ax3, self.ax4, self.ax5 = axes
        self.fig.suptitle("Exp 1 — Déploiement avec charge")
        plt.show(block=False)

    def _update_plots(self):
        with self._lock:
            t = list(self.times)
            lengths = list(self.cable_lengths)
            z_refs = list(self.z_refs)
            motor_rpms = list(self.motor_rpms)
            spool_rpms = list(self.spool_rpms)
            currents = list(self.currents)
            accels = list(self.motor_accels)
            switch_events = list(self.switch_events)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.clear()

        self.ax1.plot(t, lengths, 'g-', label='Câble réel (Hall)')
        self.ax1.plot(t, z_refs, 'r--', label='z référence')
        self.ax1.set_ylabel("Longueur (m)")
        self.ax1.legend(fontsize=8); self.ax1.grid(True, alpha=0.3)

        self.ax2.plot(t, motor_rpms, 'b-')
        self.ax2.set_ylabel("Motor RPM"); self.ax2.grid(True, alpha=0.3)

        self.ax3.scatter(t, spool_rpms, color='green', s=2)
        self.ax3.set_ylabel("Spool RPM"); self.ax3.grid(True, alpha=0.3)

        self.ax4.plot(t, currents, color='orange')
        self.ax4.set_ylabel("Current (A)"); self.ax4.grid(True, alpha=0.3)

        self.ax5.plot(t, accels, color='purple')
        self.ax5.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
        self.ax5.set_ylabel("Accel (RPM/s)"); self.ax5.set_xlabel("Time (s)")
        self.ax5.grid(True, alpha=0.3)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            for t_ev in switch_events:
                ax.axvline(x=t_ev, color='red', linewidth=1.5, linestyle='--')

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.root.after(200, self._update_plots)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
            self.connected = True
            self.conn_label.config(text="Connected ✓", foreground="green")
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
                print(f"Receive error: {e}"); break

    def _update_telemetry(self, snap):
        with self._lock:
            self.times.append(snap['t'])
            self.cable_lengths.append(snap['cable_length'])
            self.z_refs.append(snap.get('z_ref', 0))
            self.motor_rpms.append(snap['motor_rpm'])
            self.spool_rpms.append(snap['spool_rpm'])
            self.currents.append(snap.get('current', 0))
            self.switch_events = snap['switch_events']

            # Détection fin de déploiement → servo lock auto
            deploying = snap.get('deploying', False)
            if self._was_deploying and not deploying and self.auto_servo_lock.get():
                self.root.after(0, self._auto_servo_lock)
            self._was_deploying = deploying

            if len(self.motor_rpms) >= 2 and len(self.times) >= 2:
                t_list = list(self.times)
                rpm_list = list(self.motor_rpms)
                dt = t_list[-1] - t_list[-2]
                accel = (rpm_list[-1] - rpm_list[-2]) / dt if dt > 0 else 0
            else:
                accel = 0
            self.motor_accels.append(accel)

        self.root.after(0, self._update_labels, snap)

    def _update_labels(self, snap):
        self.motor_rpm_label.config(text=f"Motor RPM: {snap['motor_rpm']:.1f}")
        self.spool_rpm_label.config(text=f"Spool RPM: {snap['spool_rpm']:.1f}")
        self.length_label.config(text=f"Cable: {snap['cable_length']:.3f} m")
        self.current_label.config(text=f"Current: {snap.get('current', 0):.2f} A")
        if snap.get('switch_active'):
            self.switch_label.config(text="Switch: ON (charge accrochée)", foreground="green")
        else:
            self.switch_label.config(text="Switch: OFF (charge lâchée)", foreground="red")
            if self.switch_events:
                self.switch_time_label.config(
                    text=f"Décommutation: t={self.switch_events[-1]:.3f}s")

    def _send(self, cmd):
        if self.connected:
            try:
                self.sock.sendall((json.dumps(cmd) + '\n').encode())
            except Exception as e:
                print(f"Send error: {e}"); self.connected = False

    def _auto_servo_lock(self):
        print("Deploy terminé — servo lock auto")
        self._send({'servo_lock': True})

    def send_deploy(self):
        ramped = self.ramped_var.get()
        self._send({'deploy': {
            'z':         float(self.z_target.get()),
            'direction': self.direction.get(),
            'max_rpm':   float(self.max_rpm.get()),
            'ramped':    ramped,
            'accel':     float(self.accel.get()) if ramped else None,
            'decel':     float(self.decel.get()) if ramped else None,
        }})

    def send_reset(self):
        self._send({'reset': True})
        with self._lock:
            self.times.clear(); self.cable_lengths.clear(); self.z_refs.clear()
            self.motor_rpms.clear(); self.spool_rpms.clear()
            self.currents.clear(); self.motor_accels.clear()
            self.switch_events.clear()
        self.switch_time_label.config(text="Décommutation: --")

    def send_save_csv(self): self._send({'save_csv': True})
    def send_estop(self):    self._send({'estop': True})

    def run(self):
        self.root.after(200, self._update_plots)
        self.root.mainloop()

if __name__ == "__main__":
    app = GUIExp1()
    app.run()