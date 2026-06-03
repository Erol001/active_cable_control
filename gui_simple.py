# gui_simple.py

import tkinter as tk
from tkinter import ttk
import socket
import json
import threading
from collections import deque
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

HOST = 'raspberrypi.local'
PORT = 5005
MAX_POINTS = 500

class GUISimple:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.buffer = ""
        self._was_deploying = False
        self._sequence_active = False
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()

        self.times         = deque(maxlen=MAX_POINTS)
        self.cable_lengths = deque(maxlen=MAX_POINTS)
        self.z_refs        = deque(maxlen=MAX_POINTS)

        self.root = tk.Tk()
        self.root.title("WASP — Simple")

        self._build_gui()
        self._build_plot()
        self._schedule_plot_update()

    def _build_gui(self):
        # Connection
        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(conn, text="Connect", command=self.connect).grid(row=0, column=0, padx=5, pady=5)
        self.conn_label = ttk.Label(conn, text="Disconnected", foreground="red")
        self.conn_label.grid(row=0, column=1, padx=5)

        # Target Z + RPM
        z_frame = ttk.LabelFrame(self.root, text="Position cible")
        z_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(z_frame, text="z (m):").grid(row=0, column=0, sticky="w", padx=5)
        self.z_entry = ttk.Entry(z_frame, width=8)
        self.z_entry.insert(0, "2.0")
        self.z_entry.grid(row=0, column=1, padx=5)
        ttk.Label(z_frame, text="Max RPM:").grid(row=1, column=0, sticky="w", padx=5)
        self.rpm_entry = ttk.Entry(z_frame, width=8)
        self.rpm_entry.insert(0, "500")
        self.rpm_entry.grid(row=1, column=1, padx=5)
        ttk.Button(z_frame, text="▶  GO", command=self.send_go).grid(row=0, column=2, rowspan=2, padx=10)

        # Séquence automatique
        seq_frame = ttk.LabelFrame(self.root, text="Séquence auto")
        seq_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        ttk.Label(seq_frame, text="z cible (m):").grid(row=0, column=0, sticky="w", padx=5)
        self.seq_z = ttk.Entry(seq_frame, width=8)
        self.seq_z.insert(0, "2.0")
        self.seq_z.grid(row=0, column=1, padx=5)

        ttk.Label(seq_frame, text="Torque tension (Nm):").grid(row=1, column=0, sticky="w", padx=5)
        self.seq_torque = ttk.Entry(seq_frame, width=8)
        self.seq_torque.insert(0, "0.10")
        self.seq_torque.grid(row=1, column=1, padx=5)

        ttk.Label(seq_frame, text="Durée tension (s):").grid(row=2, column=0, sticky="w", padx=5)
        self.seq_duration = ttk.Entry(seq_frame, width=8)
        self.seq_duration.insert(0, "2.0")
        self.seq_duration.grid(row=2, column=1, padx=5)

        ttk.Label(seq_frame, text="Timeout déploi (s):").grid(row=3, column=0, sticky="w", padx=5)
        self.seq_timeout = ttk.Entry(seq_frame, width=8)
        self.seq_timeout.insert(0, "15")
        self.seq_timeout.grid(row=3, column=1, padx=5)

        tk.Button(seq_frame, text="▶▶  SÉQUENCE", command=self.start_sequence,
                  bg="darkorange", fg="white", font=("Arial", 11, "bold")).grid(
            row=4, column=0, columnspan=2, pady=6, ipadx=10)

        self.seq_status = ttk.Label(seq_frame, text="", foreground="orange",
                                    font=("Arial", 10, "bold"))
        self.seq_status.grid(row=5, column=0, columnspan=2, pady=2)

        # Servo
        servo_frame = ttk.LabelFrame(self.root, text="Servo")
        servo_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.servo_slider = ttk.Scale(servo_frame, from_=-1.0, to=1.0,
                                      orient='horizontal', length=200,
                                      command=self._on_servo_slide)
        self.servo_slider.set(0.0)
        self.servo_slider.grid(row=0, column=0, columnspan=3, padx=5, pady=5)
        self.servo_val_label = ttk.Label(servo_frame, text="Pos: 0.00")
        self.servo_val_label.grid(row=0, column=3, padx=5)
        ttk.Button(servo_frame, text="Set Min", command=self.send_servo_set_min).grid(row=1, column=0, padx=5, pady=3)
        ttk.Button(servo_frame, text="Set Max", command=self.send_servo_set_max).grid(row=1, column=1, padx=5)
        self.servo_range_label = ttk.Label(servo_frame, text="Min: -- | Max: --")
        self.servo_range_label.grid(row=2, column=0, columnspan=4, padx=5, pady=2)
        tk.Button(servo_frame, text="LOCK (min)", command=self.send_servo_lock,
                  bg="navy", fg="white", font=("Arial", 10, "bold")).grid(row=3, column=0, padx=5, pady=5)
        tk.Button(servo_frame, text="UNLOCK (max)", command=self.send_servo_unlock,
                  bg="darkgreen", fg="white", font=("Arial", 10, "bold")).grid(row=3, column=1, padx=5, pady=5)

        # Telemetry
        telem = ttk.LabelFrame(self.root, text="Telemetry")
        telem.grid(row=4, column=0, padx=10, pady=5, sticky="ew")
        self.lbl_cable  = ttk.Label(telem, text="Cable: -- m")
        self.lbl_rpm    = ttk.Label(telem, text="RPM: --")
        self.lbl_curr   = ttk.Label(telem, text="Current: -- A")
        self.lbl_deploy = ttk.Label(telem, text="", foreground="orange", font=("Arial", 10, "bold"))
        for i, l in enumerate([self.lbl_cable, self.lbl_rpm, self.lbl_curr, self.lbl_deploy]):
            l.grid(row=0, column=i, padx=8, pady=3)

        tk.Button(self.root, text="⚡  E-STOP", command=self.send_estop,
                  bg="red", fg="white", font=("Arial", 14, "bold"), height=2).grid(
            row=5, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(self.root, text="🔄  Reset", command=self.send_reset).grid(
            row=6, column=0, padx=10, pady=5, sticky="ew")

    def _build_plot(self):
        plot_frame = ttk.LabelFrame(self.root, text="Position")
        plot_frame.grid(row=7, column=0, padx=10, pady=5, sticky="ew")
        self.fig = Figure(figsize=(7, 2.5), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_ylabel("m")
        self.ax.set_xlabel("Time (s)")
        self.ax.grid(True, alpha=0.3)

        # Créer les lignes une seule fois — pas de ax.clear() à chaque update
        self.line_cable, = self.ax.plot([], [], 'g-',  linewidth=1.5, label='câble')
        self.line_zref,  = self.ax.plot([], [], 'r--', linewidth=1.2, label='z_ref')
        self.ax.legend(fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

    def _schedule_plot_update(self):
        self._update_plot()
        self.root.after(500, self._schedule_plot_update)  # 500ms au lieu de 300ms

    def _update_plot(self):
        with self._lock:
            t      = list(self.times)
            lens   = list(self.cable_lengths)
            z_refs = list(self.z_refs)

        if len(t) < 2:
            return

        # Mettre à jour les données sans recréer les objets
        self.line_cable.set_xdata(t)
        self.line_cable.set_ydata(lens)
        self.line_zref.set_xdata(t)
        self.line_zref.set_ydata(z_refs)

        self.ax.set_xlim(t[0], t[-1])
        all_vals = lens + z_refs
        ymin = min(all_vals) - 0.1
        ymax = max(all_vals) + 0.1
        if ymin == ymax:
            ymin -= 0.5
            ymax += 0.5
        self.ax.set_ylim(ymin, ymax)

        self.fig.tight_layout()
        self.canvas.draw_idle()  # Non-bloquant

    def _on_servo_slide(self, val):
        pos = float(val)
        self.servo_val_label.config(text=f"Pos: {pos:.2f}")
        self._send({'servo_pos': pos})

    # ── Séquence ─────────────────────────────────────────────────
    def start_sequence(self):
        if not self.connected:
            return
        self._sequence_active = True
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _run_sequence(self):
        import time

        z        = float(self.seq_z.get())
        torque   = float(self.seq_torque.get())
        duration = float(self.seq_duration.get())
        timeout  = float(self.seq_timeout.get())

        # Étape 1 : servo unlock (max) → lâche la charge
        self.root.after(0, self.seq_status.config, {'text': '1/4 Servo unlock...'})
        self._send({'servo_unlock': True})
        time.sleep(0.5)

        # Étape 2 : moteur IDLE → câble se déploie librement par gravité
        self.root.after(0, self.seq_status.config, {'text': '2/4 Déploiement libre...'})
        self._send({'motor_idle': True})

        # Attendre que le câble atteigne 90% de z cible
        t_start = time.time()
        while time.time() - t_start < timeout:
            with self._lock:
                current_len = list(self.cable_lengths)[-1] if self.cable_lengths else 0
            if current_len >= z * 0.9:
                break
            time.sleep(0.1)

        # Étape 3 : torque en sens retract (positif = remonte)
        self.root.after(0, self.seq_status.config, {'text': f'3/4 Tension ({torque} Nm)...'})
        self._send({'torque_start': {
            'torque':    abs(torque),
            'vel_limit': 5.0,
        }})
        time.sleep(duration)

        # Étape 4 : servo lock (min) puis stop torque
        self.root.after(0, self.seq_status.config, {'text': '4/4 Servo lock ✓'})
        self._send({'servo_lock': True})
        time.sleep(0.3)
        self._send({'torque_stop': True})

        self._sequence_active = False
        self.root.after(2000, self.seq_status.config, {'text': ''})

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
                data = self.sock.recv(4096).decode()
                if not data:
                    break
                self.buffer += data

                last_snap = None
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    try:
                        last_snap = json.loads(line)
                    except:
                        pass

                if last_snap is None:
                    continue

                snap = last_snap
                with self._lock:
                    self.times.append(snap['t'])
                    self.cable_lengths.append(snap['cable_length'])
                    self.z_refs.append(snap.get('z_ref', 0))

                self._was_deploying = snap.get('deploying', False)
                self.root.after(0, self._update_labels, snap)

            except Exception as e:
                print(f"Recv error: {e}")
                break

    def _update_labels(self, snap):
        self.lbl_cable.config(text=f"Cable: {snap['cable_length']:.3f} m")
        self.lbl_rpm.config(text=f"RPM: {snap['motor_rpm']:.0f}")
        self.lbl_curr.config(text=f"Current: {snap.get('current', 0):.2f} A")
        self.lbl_deploy.config(text="⏳ Deploying..." if snap.get('deploying') else "")
        if 'servo_min' in snap and 'servo_max' in snap:
            self.servo_range_label.config(
                text=f"Min: {snap['servo_min']:.2f} | Max: {snap['servo_max']:.2f}")

    def _send(self, cmd):
        if self.connected:
            with self._send_lock:
                try:
                    self.sock.sendall((json.dumps(cmd) + '\n').encode())
                except Exception as e:
                    print(f"Send error: {e}")
                    self.connected = False

    def send_go(self):
        self._send({'deploy': {
            'z':       float(self.z_entry.get()),
            'ramped':  True,
            'accel':   5.0,
            'decel':   3.0,
            'max_rpm': float(self.rpm_entry.get()),
        }})

    def send_reset(self):
        self._send({'reset': True})
        with self._lock:
            self.times.clear()
            self.cable_lengths.clear()
            self.z_refs.clear()
        self._was_deploying = False
        self._sequence_active = False
        self.seq_status.config(text="")

    def send_servo_set_min(self):  self._send({'servo_min': True})
    def send_servo_set_max(self):  self._send({'servo_max': True})
    def send_servo_lock(self):     self._send({'servo_lock': True})
    def send_servo_unlock(self):   self._send({'servo_unlock': True})
    def send_estop(self):
        self._sequence_active = False
        self.seq_status.config(text="")
        self._send({'estop': True})

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = GUISimple()
    app.run()