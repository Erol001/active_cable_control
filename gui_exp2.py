# gui_exp2.py — PRBS System Identification

import tkinter as tk
from tkinter import ttk
import socket
import json
import threading
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

HOST = 'raspberrypi.local'
PORT = 5005
MAX_POINTS = 500

class GUIExp2:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.buffer = ""
        self._lock = threading.Lock()

        self.times = deque(maxlen=MAX_POINTS)
        self.cable_lengths = deque(maxlen=MAX_POINTS)
        self.z_refs = deque(maxlen=MAX_POINTS)
        self.motor_rpms = deque(maxlen=MAX_POINTS)
        self.errors = deque(maxlen=MAX_POINTS)
        self.currents = deque(maxlen=MAX_POINTS)
        self.motor_accels = deque(maxlen=MAX_POINTS)
        self.switch_events = []

        self._build_gui()
        self._build_plots()

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Exp 2 — PRBS System Identification")

        # Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(row=0, column=0, padx=5, pady=5)
        self.conn_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.conn_label.grid(row=0, column=1, padx=5)

        # PRBS params
        prbs_frame = ttk.LabelFrame(self.root, text="PRBS")
        prbs_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        specs = [
            ("Centre z (m):",      "2.0",  "center"),
            ("Amplitude (m):",     "0.5",  "amplitude"),
            ("Freq min (Hz):",     "0.5",  "freq_min"),
            ("Freq max (Hz):",     "2.0",  "freq_max"),
            ("Durée preview (s):", "15",   "preview_dur"),
            ("Max RPM:",           "1000", "max_rpm"),
            ("Seed:",              "42",   "seed"),
        ]
        for i, (label, default, attr) in enumerate(specs):
            ttk.Label(prbs_frame, text=label).grid(row=i, column=0, sticky="w", padx=5, pady=2)
            e = ttk.Entry(prbs_frame, width=8); e.insert(0, default)
            e.grid(row=i, column=1, padx=5)
            setattr(self, attr, e)

        r = len(specs)

        # Ramped mode
        self.ramped_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(prbs_frame, text="Ramped mode",
                        variable=self.ramped_var,
                        command=self._toggle_ramp).grid(
            row=r, column=0, columnspan=2, sticky="w", padx=5, pady=(6, 0))
        r += 1

        self.lbl_accel = ttk.Label(prbs_frame, text="Accel (tr/s²):")
        self.lbl_decel = ttk.Label(prbs_frame, text="Decel (tr/s²):")
        self.accel = ttk.Entry(prbs_frame, width=8); self.accel.insert(0, "10.0")
        self.decel = ttk.Entry(prbs_frame, width=8); self.decel.insert(0, "10.0")
        self.lbl_accel.grid(row=r,   column=0, sticky="w", padx=5)
        self.accel.grid(row=r,       column=1, padx=5); r += 1
        self.lbl_decel.grid(row=r,   column=0, sticky="w", padx=5)
        self.decel.grid(row=r,       column=1, padx=5); r += 1

        btn_frame2 = ttk.Frame(prbs_frame)
        btn_frame2.grid(row=r, column=0, columnspan=2, pady=6)
        ttk.Button(btn_frame2, text="👁  Preview", command=self.preview_prbs).grid(row=0, column=0, padx=4)
        ttk.Button(btn_frame2, text="▶  Start",   command=self.send_start_prbs).grid(row=0, column=1, padx=4)
        ttk.Button(btn_frame2, text="■  Stop",    command=self.send_stop_prbs).grid(row=0, column=2, padx=4)

        # RMS
        self.rms_label = ttk.Label(self.root, text="RMS error: -- m", font=("Arial", 11, "bold"))
        self.rms_label.grid(row=2, column=0, padx=10, pady=5)

        # Telemetry
        telem_frame = ttk.LabelFrame(self.root, text="Telemetry")
        telem_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.motor_rpm_label = ttk.Label(telem_frame, text="Motor RPM: --")
        self.motor_rpm_label.grid(row=0, column=0, padx=5)
        self.length_label = ttk.Label(telem_frame, text="Cable: -- m")
        self.length_label.grid(row=0, column=1, padx=5)
        self.current_label = ttk.Label(telem_frame, text="Current: -- A")
        self.current_label.grid(row=0, column=2, padx=5)

        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(btn_frame, text="Reset",    command=self.send_reset).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Save CSV", command=self.send_save_csv).grid(row=0, column=1, padx=5)

        tk.Button(self.root, text="⚡  E-STOP", command=self.send_estop,
                  bg="red", fg="white", font=("Arial", 14, "bold"), height=2).grid(
            row=5, column=0, padx=10, pady=10, sticky="ew")

    def _toggle_ramp(self):
        state = "normal" if self.ramped_var.get() else "disabled"
        self.accel.config(state=state)
        self.decel.config(state=state)

    # ── PRBS preview ─────────────────────────────────────────────
    def _generate_prbs(self, center, amplitude, freq_min, freq_max, duration=15, fs=10, seed=42):
        rng = np.random.RandomState(seed)
        t = np.arange(0, duration, 1 / fs)
        min_hold = max(1, int(fs / freq_max))
        max_hold = max(min_hold + 1, int(fs / freq_min))
        signal = np.full(len(t), center - amplitude, dtype=float)
        val, i = center - amplitude, 0
        while i < len(t):
            hold = rng.randint(min_hold, max_hold + 1)
            val = center + amplitude if val == center - amplitude else center - amplitude
            signal[i:min(i + hold, len(t))] = val
            i += hold
        return t, signal

    def preview_prbs(self):
        try:
            c    = float(self.center.get())
            a    = float(self.amplitude.get())
            fmin = float(self.freq_min.get())
            fmax = float(self.freq_max.get())
            dur  = float(self.preview_dur.get())
            seed = int(self.seed.get())
        except ValueError as e:
            print(f"Preview error: {e}"); return

        t, sig = self._generate_prbs(c, a, fmin, fmax, duration=dur, seed=seed)

        fig, ax = plt.subplots(figsize=(9, 3))
        ax.step(t, sig, where='post', color='royalblue', lw=1.5, label='PRBS')
        ax.axhline(c,     color='gray',   ls='--', lw=0.8, label=f'centre = {c} m')
        ax.axhline(c + a, color='tomato', ls=':',  lw=0.8, label=f'+{a} m')
        ax.axhline(c - a, color='tomato', ls=':',  lw=0.8, label=f'−{a} m')
        ax.fill_between(t, c - a, c + a, alpha=0.07, color='royalblue')
        ax.set_xlabel("Time (s)"); ax.set_ylabel("z (m)")
        ax.set_title(
            f"PRBS Preview — centre={c} m | ±{a} m | [{fmin}–{fmax}] Hz | seed={seed}")
        ax.legend(fontsize=8, loc='upper right'); ax.grid(True, alpha=0.3)
        fig.tight_layout()
        plt.show(block=False)

    # ── Plots ─────────────────────────────────────────────────────
    def _build_plots(self):
        self.fig, axes = plt.subplots(5, 1, figsize=(9, 13))
        self.ax1, self.ax2, self.ax3, self.ax4, self.ax5 = axes
        self.fig.suptitle("Exp 2 — PRBS System Identification")
        plt.show(block=False)

    def _update_plots(self):
        with self._lock:
            t = list(self.times)
            lengths = list(self.cable_lengths)
            z_refs = list(self.z_refs)
            motor_rpms = list(self.motor_rpms)
            errors = list(self.errors)
            currents = list(self.currents)
            accels = list(self.motor_accels)

        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.clear()

        self.ax1.step(t, z_refs, 'r--', where='post', label='z référence (PRBS)')
        self.ax1.plot(t, lengths, 'g-', label='z mesuré (Hall)')
        self.ax1.set_ylabel("Longueur (m)")
        self.ax1.legend(fontsize=8); self.ax1.grid(True, alpha=0.3)

        self.ax2.plot(t, errors, color='orange')
        self.ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
        self.ax2.set_ylabel("Erreur (m)"); self.ax2.grid(True, alpha=0.3)

        self.ax3.plot(t, motor_rpms, 'b-')
        self.ax3.set_ylabel("Motor RPM"); self.ax3.grid(True, alpha=0.3)

        self.ax4.plot(t, currents, color='orange')
        self.ax4.set_ylabel("Current (A)"); self.ax4.grid(True, alpha=0.3)

        self.ax5.plot(t, accels, color='purple')
        self.ax5.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
        self.ax5.set_ylabel("Accel (RPM/s)"); self.ax5.set_xlabel("Time (s)")
        self.ax5.grid(True, alpha=0.3)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        if errors:
            rms = (sum(e**2 for e in errors) / len(errors)) ** 0.5
            self.rms_label.config(text=f"RMS error: {rms:.4f} m")

        self.root.after(200, self._update_plots)

    # ── Network ───────────────────────────────────────────────────
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
                if not data: break
                self.buffer += data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    snap = json.loads(line)
                    self._update_telemetry(snap)
            except Exception as e:
                print(f"Receive error: {e}"); break

    def _update_telemetry(self, snap):
        z_ref  = snap.get('z_ref', 0)
        z_real = snap['cable_length']
        error  = z_ref - z_real
        with self._lock:
            self.times.append(snap['t'])
            self.cable_lengths.append(z_real)
            self.z_refs.append(z_ref)
            self.motor_rpms.append(snap['motor_rpm'])
            self.errors.append(error)
            self.currents.append(snap.get('current', 0))
            self.switch_events = snap.get('switch_events', [])

            if len(self.motor_rpms) >= 2 and len(self.times) >= 2:
                t_list = list(self.times); rpm_list = list(self.motor_rpms)
                dt = t_list[-1] - t_list[-2]
                accel = (rpm_list[-1] - rpm_list[-2]) / dt if dt > 0 else 0
            else:
                accel = 0
            self.motor_accels.append(accel)

        self.root.after(0, self._update_labels, snap, z_real)

    def _update_labels(self, snap, z_real):
        self.motor_rpm_label.config(text=f"Motor RPM: {snap['motor_rpm']:.1f}")
        self.length_label.config(text=f"Cable: {z_real:.3f} m")
        self.current_label.config(text=f"Current: {snap.get('current', 0):.2f} A")

    def _send(self, cmd):
        if self.connected:
            try:
                self.sock.sendall((json.dumps(cmd) + '\n').encode())
            except Exception as e:
                print(f"Send error: {e}"); self.connected = False

    def send_start_prbs(self):
        ramped = self.ramped_var.get()
        if ramped:
            self._send({'set_traj_params': {
                'max_rpm': float(self.max_rpm.get()),
                'accel': float(self.accel.get()),
                'decel': float(self.decel.get()),
            }})
        self._send({'start_prbs': {
            'center':    float(self.center.get()),
            'amplitude': float(self.amplitude.get()),
            'freq_min':  float(self.freq_min.get()),
            'freq_max':  float(self.freq_max.get()),
            'seed':      int(self.seed.get()),
            'ramped':    ramped,
        }})

    def send_stop_prbs(self): self._send({'stop_prbs': True})

    def send_reset(self):
        self._send({'reset': True})
        with self._lock:
            self.times.clear(); self.cable_lengths.clear(); self.z_refs.clear()
            self.motor_rpms.clear(); self.errors.clear()
            self.currents.clear(); self.motor_accels.clear()

    def send_save_csv(self): self._send({'save_csv': True})
    def send_estop(self):    self._send({'estop': True})

    def run(self):
        self.root.after(200, self._update_plots)
        self.root.mainloop()

if __name__ == "__main__":
    app = GUIExp2()
    app.run()