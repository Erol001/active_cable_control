# test_motor_gui.py

import tkinter as tk
from tkinter import ttk
import socket
import json
import threading

HOST = 'raspberrypi.local'
PORT = 5006

class MotorTestGUI:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.buffer = ""
        self._build_gui()

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Motor Test")

        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        ttk.Button(conn_frame, text="Connect", command=self.connect).grid(row=0, column=0, padx=5, pady=5)
        self.conn_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.conn_label.grid(row=0, column=1, padx=5)

        ctrl_frame = ttk.LabelFrame(self.root, text="Motor Control")
        ctrl_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(ctrl_frame, text="Target RPM:").grid(row=0, column=0, padx=5, pady=5)
        self.rpm_entry = ttk.Entry(ctrl_frame, width=10)
        self.rpm_entry.insert(0, "100")
        self.rpm_entry.grid(row=0, column=1, padx=5)

        ttk.Button(ctrl_frame, text="Enable", command=self.enable).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(ctrl_frame, text="Send RPM", command=self.send_rpm).grid(row=1, column=1, padx=5)
        ttk.Button(ctrl_frame, text="Stop", command=self.stop).grid(row=1, column=2, padx=5)

        telem_frame = ttk.LabelFrame(self.root, text="Telemetry")
        telem_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        self.vbus_label = ttk.Label(telem_frame, text="Vbus: --")
        self.vbus_label.grid(row=0, column=0, padx=5)
        self.vel_label = ttk.Label(telem_frame, text="RPM: --")
        self.vel_label.grid(row=0, column=1, padx=5)
        self.current_label = ttk.Label(telem_frame, text="Current: --")
        self.current_label.grid(row=0, column=2, padx=5)

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
            self.connected = True
            self.conn_label.config(text="Connected", foreground="green")
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
                    telem = json.loads(line)
                    self.vbus_label.config(text=f"Vbus: {telem['vbus']:.2f}V")
                    self.vel_label.config(text=f"RPM: {telem['vel']:.1f}")
                    self.current_label.config(text=f"Current: {telem['current']:.2f}A")
            except Exception as e:
                print(f"Receive error: {e}")
                break

    def _send(self, cmd):
        if self.connected:
            try:
                self.sock.sendall((json.dumps(cmd) + '\n').encode())
            except Exception as e:
                print(f"Send error: {e}")

    def enable(self):
        self._send({'action': 'enable'})

    def send_rpm(self):
        rpm = float(self.rpm_entry.get())
        self._send({'action': 'set_rpm', 'rpm': rpm})

    def stop(self):
        self._send({'action': 'stop'})

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    app = MotorTestGUI()
    app.run()
    