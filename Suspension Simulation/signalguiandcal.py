# =============================================================================
#  SUSPEN-SIM  —  Single-Servo GUI Edition (Simplified)
#  File   : suspen_sim_gui.py
#
#  ARCHITECTURE
#  ────────────
#  1 GUI  →  1 Arduino  →  1 Servo (D9)
#  Single serial connection. One servo controlled from one board.
#  Direction inversion handled in Arduino firmware (DIR1).
#
#  USER CONTROLS
#  ──────────────
#  Only Mass (M), Spring constant (K), and Damping (B) are exposed.
#  Signal parameters (amplitude, frequency, etc.) are fixed at values
#  tuned to produce clearly visible servo motion for each signal type,
#  representing short road-bump / road-condition events.
#
#  INSTALL
#  ───────
#  pip install pyserial numpy customtkinter matplotlib
# =============================================================================

import threading
import time
import math
import serial
import serial.tools.list_ports
import numpy as np

import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# =============================================================================
#  FIXED SIGNAL PARAMETERS  (tuned for short, visible "road bump" events)
#  Combined with the servo's SCALE_FACTOR (45 deg/m), every signal
#  produces a clearly visible swing on the servo horn (roughly
#  20-90 degrees of motion) without exceeding mechanical limits,
#  while finishing quickly so multiple road conditions can be tested
#  back-to-back.
# =============================================================================
FIXED_AMP   = 1.2     # base amplitude (m or N depending on signal)
FIXED_FREQ  = 1.5     # Hz, for sine signal (wavy road)
FIXED_DUR   = 0.03    # s, impulse pulse duration (sharp pothole)
FIXED_RRATE = 2.5     # N/s, ramp rate (curb / slope)
FIXED_F1    = 8.0     # Hz, chirp end frequency (rough patch sweep)
FIXED_T     = 3.0     # s, chirp duration (short rough-patch sweep)


# =============================================================================
#  PHYSICS ENGINE
# =============================================================================
class MassSpringDamper:
    def __init__(self, M=1.0, K=4.0, B=2.0, dt=0.001):
        self.M = M; self.K = K; self.B = B; self.dt = dt
        self.reset()

    def reset(self):
        self.t = 0.0; self.x = 0.0; self.v = 0.0; self.acc = 0.0

    def _force(self, t, sig, p):
        if sig == "step":
            return 0.0
        elif sig == "impulse":
            return p.get("amp", 20.0) if t <= p.get("dur", 0.05) else 0.0
        elif sig == "sine":
            return p.get("amp", 3.0) * math.sin(p.get("omega", 2.0) * t)
        elif sig == "ramp":
            rate = p.get("rate", 1.0)
            return min(rate * t, rate * p.get("tmax", 4.0))
        elif sig == "chirp":
            f0 = p.get("f0", 0.1); f1 = p.get("f1", 6.0); T = p.get("T", 15.0)
            beta = (f1 - f0) / T
            return p.get("amp", 2.0) * math.sin(2*math.pi*(f0*t + 0.5*beta*t**2))
        return 0.0

    def _deriv(self, t, x, v, sig, p):
        F = self._force(t, sig, p)
        return v, (F - self.B*v - self.K*x) / self.M

    def step(self, sig="step", params=None):
        if params is None: params = {}
        dt = self.dt; t, x, v = self.t, self.x, self.v
        k1x, k1v = self._deriv(t,        x,           v,           sig, params)
        k2x, k2v = self._deriv(t+dt/2,   x+k1x*dt/2, v+k1v*dt/2,  sig, params)
        k3x, k3v = self._deriv(t+dt/2,   x+k2x*dt/2, v+k2v*dt/2,  sig, params)
        k4x, k4v = self._deriv(t+dt,     x+k3x*dt,   v+k3v*dt,    sig, params)
        self.x  += (dt/6)*(k1x + 2*k2x + 2*k3x + k4x)
        self.v  += (dt/6)*(k1v + 2*k2v + 2*k3v + k4v)
        self.t  += dt
        self.acc = self._deriv(self.t, self.x, self.v, sig, params)[1]
        return self.x, self.v, self.acc

    @property
    def omega_n(self): return math.sqrt(self.K / self.M)

    @property
    def zeta(self):
        d = 2*math.sqrt(self.M * self.K)
        return self.B / d if d > 0 else 0.0

    @property
    def omega_d(self):
        z = self.zeta
        return self.omega_n * math.sqrt(max(0, 1 - z**2)) if z < 1 else 0.0

    @property
    def settling_time(self):
        z = self.zeta
        return 4 / (z * self.omega_n) if z > 0.01 else float("inf")

    @property
    def damping_label(self):
        z = self.zeta
        if abs(z - 1) < 0.05: return "Critically Damped"
        return "Underdamped" if z < 1 else "Overdamped"


# =============================================================================
#  SERIAL LINK  — single connection to one Arduino driving one servo
# =============================================================================
class SerialLink:
    def __init__(self):
        self._ser = None
        self._port = None

    def connect(self, port, baud=115200):
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self._port = port
            time.sleep(2.0)
            self._ser.flushInput()
            return True
        except Exception:
            return False

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self.send_position(0.0)
            time.sleep(0.1)
            self._ser.close()
        self._ser = None

    def send_position(self, x):
        self._write(f"X:{x:.4f}\n")

    def send_params(self, M, K, B):
        self._write(f"P:{M:.2f},{K:.2f},{B:.2f}\n")

    def _write(self, msg):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(msg.encode("ascii"))
            except Exception:
                pass

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    @property
    def connected(self):
        return self._ser is not None and self._ser.is_open

    @property
    def port(self):
        return self._port or "---"


# =============================================================================
#  SIMULATION RUNNER
# =============================================================================
class SimulationRunner:
    SERIAL_INTERVAL = 0.02
    SETTLE_TOL      = 1e-4
    MAX_SIM_TIME    = 90.0
    HISTORY_LEN     = 600

    def __init__(self, system: MassSpringDamper, link: SerialLink):
        self.system    = system
        self.link      = link          # single SerialLink
        self._thread   = None
        self._stop_ev  = threading.Event()
        self.current_x = 0.0
        self.current_v = 0.0
        self.current_a = 0.0
        self.finished  = False
        self.hist_t = []; self.hist_x = []
        self.hist_v = []; self.hist_a = []
        self.hist_F = []

    def start(self, sig, params, init_x=0.0, init_v=0.0):
        self._stop_ev.clear(); self.finished = False
        self.hist_t.clear(); self.hist_x.clear()
        self.hist_v.clear(); self.hist_a.clear()
        self.hist_F.clear()
        self.system.reset()
        self.system.x = init_x; self.system.v = init_v
        self._thread = threading.Thread(
            target=self._run, args=(sig, params), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_ev.set()
        if self._thread: self._thread.join(timeout=2.0)
        self.link.send_position(0.0)

    def _run(self, sig, params):
        dt = self.system.dt
        wall_start    = time.perf_counter()
        last_serial_t = wall_start

        while not self._stop_ev.is_set():
            wall_now     = time.perf_counter()
            wall_elapsed = wall_now - wall_start
            steps_needed = min(50, max(0,
                int(wall_elapsed / dt) - int(self.system.t / dt)))

            for _ in range(steps_needed):
                x, v, a = self.system.step(sig, params)

            self.current_x = self.system.x
            self.current_v = self.system.v
            self.current_a = self.system.acc

            if (wall_now - last_serial_t) >= self.SERIAL_INTERVAL:
                self.link.send_position(self.current_x)
                last_serial_t = wall_now

                self.hist_t.append(self.system.t)
                self.hist_x.append(self.current_x)
                self.hist_v.append(self.current_v)
                self.hist_a.append(self.current_a)
                F_out = (self.system.M * self.current_a
                         + self.system.B * self.current_v
                         + self.system.K * self.current_x)
                self.hist_F.append(F_out)
                if len(self.hist_t) > self.HISTORY_LEN:
                    self.hist_t.pop(0); self.hist_x.pop(0)
                    self.hist_v.pop(0); self.hist_a.pop(0)
                    self.hist_F.pop(0)

            settled = (sig in ("step", "impulse", "ramp")
                       and abs(self.current_x) < self.SETTLE_TOL
                       and abs(self.current_v) < self.SETTLE_TOL
                       and self.system.t > 0.5)
            if settled or self.system.t > self.MAX_SIM_TIME:
                break
            time.sleep(0.0004)

        self.link.send_position(0.0)
        self.finished = True


# =============================================================================
#  GUI
# =============================================================================
DARK_BG     = "#0d1117"
PANEL_BG    = "#161b22"
ACCENT      = "#00d4ff"
ACCENT2     = "#ff6b35"
ACCENT3     = "#7ee787"
TEXT_DIM    = "#8b949e"
TEXT_BRIGHT = "#e6edf3"

SIGNAL_TYPES = ["step", "impulse", "sine", "ramp", "chirp"]


class SuspenSimApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("SUSPEN-SIM  ·  Single-Servo  |  Single Arduino")
        self.geometry("1320x860")
        self.minsize(1100, 720)
        self.configure(fg_color=DARK_BG)

        self.system = MassSpringDamper()
        self.link   = SerialLink()                      # ONE connection
        self.runner = SimulationRunner(self.system, self.link)

        self._build_ui()
        self._refresh_ports()
        self._ui_update_loop()

    # =========================================================================
    #  UI BUILD
    # =========================================================================
    def _build_ui(self):
        # Top bar
        top = ctk.CTkFrame(self, fg_color=PANEL_BG, corner_radius=0, height=56)
        top.pack(fill="x", side="top")
        top.pack_propagate(False)

        ctk.CTkLabel(top, text="⚙  SUSPEN-SIM",
                     font=("Courier New", 20, "bold"),
                     text_color=ACCENT).pack(side="left", padx=24)
        ctk.CTkLabel(top, text="Single-Servo · Single Arduino · Physics Simulator",
                     font=("Courier New", 11),
                     text_color=TEXT_DIM).pack(side="left")

        self.lbl_status = ctk.CTkLabel(top, text="● IDLE",
                                       font=("Courier New", 12, "bold"),
                                       text_color=TEXT_DIM)
        self.lbl_status.pack(side="right", padx=24)

        # Body: 3 columns
        body = ctk.CTkFrame(self, fg_color=DARK_BG)
        body.pack(fill="both", expand=True, padx=12, pady=8)
        body.columnconfigure(0, weight=2, minsize=280)
        body.columnconfigure(1, weight=5)
        body.columnconfigure(2, weight=2, minsize=260)
        body.rowconfigure(0, weight=1)

        self._build_left(body) .grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._build_mid(body)  .grid(row=0, column=1, sticky="nsew", padx=6)
        self._build_right(body).grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        self._build_bottom()

    # ── Left: physics sliders (M, K, B only) ───────────────────────────────────
    def _build_left(self, parent):
        frame = ctk.CTkFrame(parent, fg_color=PANEL_BG, corner_radius=12)

        ctk.CTkLabel(frame, text="SYSTEM PARAMETERS",
                     font=("Courier New", 12, "bold"),
                     text_color=ACCENT).pack(pady=(16, 4), padx=16, anchor="w")
        self._sep(frame)

        self.sv_M = ctk.DoubleVar(value=1.0)
        self.sv_K = ctk.DoubleVar(value=4.0)
        self.sv_B = ctk.DoubleVar(value=2.0)

        # Slider ranges are generous; users can also type any value directly
        # (including values outside these ranges) in the entry box.
        self._param_row(frame, "Mass  M",    "kg",   self.sv_M, 0.1, 50.0,  self._on_param)
        self._param_row(frame, "Spring  K",  "N/m",  self.sv_K, 0.1, 200.0, self._on_param)
        self._param_row(frame, "Damping  B", "Ns/m", self.sv_B, 0.0, 200.0, self._on_param)

        self._sep(frame)

        ctk.CTkLabel(frame,
                     text="Use the slider or type a value\ndirectly in the box, then press\nEnter or click away to apply.\n\nSignal amplitude, frequency, and\ndurations are fixed at values\ntuned for short, clearly visible\nroad-bump servo motion.",
                     font=("Courier New", 9),
                     text_color=TEXT_DIM,
                     justify="left").pack(pady=(8, 16), padx=16, anchor="w")

        return frame

    # ── Mid: waveform ─────────────────────────────────────────────────────────
    def _build_mid(self, parent):
        frame = ctk.CTkFrame(parent, fg_color=PANEL_BG, corner_radius=12)

        ctk.CTkLabel(frame, text="LIVE SIGNALS  —  x(t)  &  F(t) = M·a + B·v + K·x",
                     font=("Courier New", 12, "bold"),
                     text_color=ACCENT).pack(pady=(16, 4), padx=16, anchor="w")

        self.fig = Figure(figsize=(7, 5.5), dpi=100, facecolor=DARK_BG)
        self.fig.subplots_adjust(hspace=0.18, left=0.10, right=0.97,
                                 top=0.96, bottom=0.10)

        # Two stacked plots: x(t) = servo position signal, F(t) = equation output
        self.ax_x = self.fig.add_subplot(2, 1, 1, facecolor=DARK_BG)
        self.ax_f = self.fig.add_subplot(2, 1, 2, facecolor=DARK_BG)

        self.ax_x.tick_params(colors=TEXT_DIM, labelsize=9)
        self.ax_x.set_ylabel("x(t)  [m]  (servo signal)", color=ACCENT, fontsize=9, labelpad=6)
        self.ax_x.set_xticklabels([])
        for spine in self.ax_x.spines.values(): spine.set_color("#30363d")
        self.ax_x.grid(True, color="#21262d", linewidth=0.6)
        self.ax_x.axhline(0, color="#30363d", linewidth=0.8)

        self.ax_f.tick_params(colors=TEXT_DIM, labelsize=9)
        self.ax_f.set_ylabel("F(t)  [N]  =  M·a+B·v+K·x", color=ACCENT2, fontsize=9, labelpad=6)
        self.ax_f.set_xlabel("time  [s]", color=TEXT_DIM, fontsize=10)
        for spine in self.ax_f.spines.values(): spine.set_color("#30363d")
        self.ax_f.grid(True, color="#21262d", linewidth=0.6)
        self.ax_f.axhline(0, color="#30363d", linewidth=0.8)

        self.line_x, = self.ax_x.plot([], [], color=ACCENT, lw=2.0)
        self.line_f, = self.ax_f.plot([], [], color=ACCENT2, lw=1.6)

        self.canvas_mpl = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas_mpl.get_tk_widget().pack(fill="both", expand=True,
                                             padx=8, pady=(0, 8))

        # Metrics strip
        mf = ctk.CTkFrame(frame, fg_color="#0d1117", corner_radius=8)
        mf.pack(fill="x", padx=8, pady=(0, 12))
        self.lbl_zeta = self._metric(mf, "ζ (zeta)",   "0.5000")
        self.lbl_wn   = self._metric(mf, "ωₙ (rad/s)", "2.000")
        self.lbl_wd   = self._metric(mf, "ωd (rad/s)", "1.732")
        self.lbl_ts   = self._metric(mf, "ts (s)",      "4.00")
        self.lbl_damp = self._metric(mf, "State",       "Underdamped")
        self.lbl_simx = self._metric(mf, "x (m)",       "0.0000")

        return frame

    # ── Right: single Arduino connection + signal type ────────────────────────
    def _build_right(self, parent):
        frame = ctk.CTkFrame(parent, fg_color=PANEL_BG, corner_radius=12)

        ctk.CTkLabel(frame, text="ARDUINO CONNECTION",
                     font=("Courier New", 12, "bold"),
                     text_color=ACCENT).pack(pady=(16, 4), padx=16, anchor="w")
        self._sep(frame)

        # Single port selector
        sf = ctk.CTkFrame(frame, fg_color="#0d1117", corner_radius=8)
        sf.pack(fill="x", padx=12, pady=6)

        ctk.CTkLabel(sf, text="Arduino  (Servo → D9)",
                     font=("Courier New", 10, "bold"),
                     text_color=TEXT_BRIGHT).pack(anchor="w", padx=10, pady=(8, 2))

        self.cb_port = ctk.CTkComboBox(sf, state="readonly", width=200,
                                        font=("Courier New", 11),
                                        fg_color="#161b22",
                                        border_color="#30363d",
                                        button_color="#21262d")
        self.cb_port.pack(padx=10, pady=4)

        row = ctk.CTkFrame(sf, fg_color="#0d1117")
        row.pack(fill="x", padx=10, pady=(2, 8))

        self.led_con = ctk.CTkLabel(row, text="●", font=("Arial", 16),
                                     text_color=TEXT_DIM)
        self.led_con.pack(side="left")

        self.btn_con = ctk.CTkButton(
            row, text="Connect", width=100,
            font=("Courier New", 11),
            fg_color="#21262d", hover_color="#2d333b",
            border_color=ACCENT, border_width=1,
            command=self._toggle_connect)
        self.btn_con.pack(side="left", padx=8)

        self.lbl_port = ctk.CTkLabel(row, text="",
                                      font=("Courier New", 9),
                                      text_color=TEXT_DIM)
        self.lbl_port.pack(side="left", padx=4)

        self._sep(frame)

        # Calibration info
        cal = ctk.CTkFrame(frame, fg_color="#0d1117", corner_radius=8)
        cal.pack(fill="x", padx=12, pady=6)
        ctk.CTkLabel(cal, text="Servo calibration (edit in .ino)",
                     font=("Courier New", 10, "bold"),
                     text_color=TEXT_DIM).pack(anchor="w", padx=10, pady=(8, 2))
        ctk.CTkLabel(cal,
                     text="DIR1 = true  (normal)\nOFFSET1 = 0",
                     font=("Courier New", 9),
                     text_color=TEXT_DIM,
                     justify="left").pack(anchor="w", padx=10, pady=(0, 8))

        self._sep(frame)

        ctk.CTkButton(frame, text="⟳  Refresh Ports",
                      font=("Courier New", 11),
                      fg_color="#21262d", hover_color="#2d333b",
                      command=self._refresh_ports
                      ).pack(pady=6, padx=12, fill="x")

        self._sep(frame)

        # Signal type
        ctk.CTkLabel(frame, text="ROAD CONDITION", 
                     font=("Courier New", 12, "bold"),
                     text_color=ACCENT).pack(pady=(8, 4), padx=16, anchor="w")

        self.sig_var = ctk.StringVar(value="step")
        sig_descs = {
            "step":    "Step — sudden curb / ledge",
            "impulse": "Impulse — sharp pothole hit",
            "sine":    "Sine — wavy/corrugated road",
            "ramp":    "Ramp — gradual slope / incline",
            "chirp":   "Chirp — rough patch sweep",
        }
        for s in SIGNAL_TYPES:
            ctk.CTkRadioButton(
                frame, text=sig_descs[s],
                variable=self.sig_var, value=s,
                font=("Courier New", 10),
                fg_color=ACCENT, hover_color=ACCENT,
                text_color=TEXT_BRIGHT
            ).pack(anchor="w", padx=20, pady=3)

        return frame

    # ── Bottom: run / stop bar ────────────────────────────────────────────────
    def _build_bottom(self):
        bar = ctk.CTkFrame(self, fg_color=PANEL_BG, corner_radius=0, height=64)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        self.btn_run = ctk.CTkButton(
            bar, text="▶  RUN SIMULATION",
            font=("Courier New", 14, "bold"),
            fg_color=ACCENT, hover_color="#00a8cc",
            text_color=DARK_BG, width=200, height=40,
            command=self._run_sim)
        self.btn_run.pack(side="left", padx=24, pady=12)

        self.btn_stop = ctk.CTkButton(
            bar, text="■  STOP",
            font=("Courier New", 13, "bold"),
            fg_color="#f85149", hover_color="#da3633",
            text_color="white", width=120, height=40,
            command=self._stop_sim, state="disabled")
        self.btn_stop.pack(side="left", padx=8, pady=12)

        self.lbl_con_status = ctk.CTkLabel(
            bar, text="No Arduino connected",
            font=("Courier New", 11), text_color=TEXT_DIM)
        self.lbl_con_status.pack(side="left", padx=24)

        self.lbl_time = ctk.CTkLabel(
            bar, text="t = 0.000 s",
            font=("Courier New", 13), text_color=TEXT_DIM)
        self.lbl_time.pack(side="right", padx=24)

    # =========================================================================
    #  HELPERS
    # =========================================================================
    def _sep(self, parent):
        ctk.CTkFrame(parent, fg_color="#21262d", height=1,
                     corner_radius=0).pack(fill="x", padx=12, pady=4)

    def _param_row(self, parent, label, unit, var, lo, hi, cmd):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", padx=12, pady=3)

        top_row = ctk.CTkFrame(row, fg_color="transparent")
        top_row.pack(fill="x")

        ctk.CTkLabel(top_row, text=label, width=100,
                     font=("Courier New", 10),
                     text_color=TEXT_BRIGHT, anchor="w").pack(side="left")

        ctk.CTkLabel(top_row, text=unit, width=36,
                     font=("Courier New", 9),
                     text_color=TEXT_DIM).pack(side="right")

        entry = ctk.CTkEntry(top_row, width=64,
                              font=("Courier New", 10, "bold"),
                              fg_color="#0d1117",
                              border_color="#30363d",
                              text_color=ACCENT,
                              justify="right")
        entry.pack(side="right", padx=(4, 4))
        entry.insert(0, f"{var.get():.2f}")

        # Slider may extend below 0 or above hi if the user types a value
        # outside the nominal range; in that case the slider just clamps
        # to its end while the entry box keeps the exact value.
        slider = ctk.CTkSlider(row, variable=var, from_=lo, to=hi,
                                fg_color="#21262d", progress_color=ACCENT,
                                button_color=ACCENT, button_hover_color="#00a8cc")
        slider.pack(fill="x", expand=True, pady=(4, 0))

        def _from_slider(v):
            entry.delete(0, "end")
            entry.insert(0, f"{float(v):.2f}")
            if cmd: cmd()

        def _from_entry(_event=None):
            try:
                val = float(entry.get())
            except ValueError:
                entry.delete(0, "end")
                entry.insert(0, f"{var.get():.2f}")
                return
            var.set(val)
            # Move slider handle if within range; otherwise clamp visually
            clamped = max(lo, min(hi, val))
            slider.set(clamped)
            if cmd: cmd()

        slider.configure(command=_from_slider)
        entry.bind("<Return>", _from_entry)
        entry.bind("<FocusOut>", _from_entry)

        return row


    def _metric(self, parent, label, init):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(side="left", expand=True, padx=6, pady=6)
        ctk.CTkLabel(f, text=label, font=("Courier New", 8),
                     text_color=TEXT_DIM).pack()
        lbl = ctk.CTkLabel(f, text=init,
                            font=("Courier New", 11, "bold"),
                            text_color=TEXT_BRIGHT)
        lbl.pack()
        return lbl

    # =========================================================================
    #  CALLBACKS
    # =========================================================================
    def _on_param(self):
        self.system.M = max(0.01, self.sv_M.get())
        self.system.K = max(0.01, self.sv_K.get())
        self.system.B = max(0.0,  self.sv_B.get())
        self._update_metrics()

    def _update_metrics(self):
        s = self.system
        z = s.zeta
        col = (ACCENT3 if "Critical" in s.damping_label else
               ACCENT2 if "Under"    in s.damping_label else "#f85149")
        self.lbl_zeta.configure(text=f"{z:.4f}", text_color=col)
        self.lbl_wn.configure(  text=f"{s.omega_n:.3f}")
        self.lbl_wd.configure(  text=f"{s.omega_d:.3f}")
        ts = s.settling_time
        self.lbl_ts.configure(  text="∞" if ts == float("inf") else f"{ts:.2f}")
        self.lbl_damp.configure(text=s.damping_label, text_color=col)

    def _refresh_ports(self):
        ports = SerialLink.list_ports() or ["(none)"]
        self.cb_port.configure(values=ports)
        self.cb_port.set(ports[0])

    def _toggle_connect(self):
        if self.link.connected:
            self.link.disconnect()
            self.led_con.configure(text_color=TEXT_DIM)
            self.btn_con.configure(text="Connect")
            self.lbl_port.configure(text="")
            self.lbl_con_status.configure(
                text="No Arduino connected", text_color=TEXT_DIM)
        else:
            port = self.cb_port.get()
            if port == "(none)": return
            if self.link.connect(port):
                self.led_con.configure(text_color=ACCENT3)
                self.btn_con.configure(text="Disconnect")
                self.lbl_port.configure(text=port)
                self.lbl_con_status.configure(
                    text=f"Connected  {port}  — Servo → D9",
                    text_color=ACCENT3)
                self.link.send_params(
                    self.system.M, self.system.K, self.system.B)
            else:
                self.led_con.configure(text_color="#f85149")
                self.lbl_con_status.configure(
                    text="Connection failed", text_color="#f85149")

    def _build_params(self):
        sig = self.sig_var.get()

        # Fixed signal parameters, tuned to produce visible, short-duration
        # servo motion that mimics realistic road-bump events, regardless
        # of M, K, B chosen by the user.
        init_x = 0.0; params = {}
        if sig == "step":
            init_x = FIXED_AMP
        elif sig == "impulse":
            params = {"amp": FIXED_AMP * 10, "dur": FIXED_DUR}
        elif sig == "sine":
            params = {"amp": FIXED_AMP * 2, "omega": 2*math.pi*FIXED_FREQ}
        elif sig == "ramp":
            params = {"rate": FIXED_RRATE, "tmax": 2.0}
        elif sig == "chirp":
            params = {"f0": 0.5, "f1": FIXED_F1, "T": FIXED_T,
                      "amp": FIXED_AMP * 1.5}
        return sig, params, init_x, 0.0

    def _run_sim(self):
        self._on_param()
        sig, params, ix, iv = self._build_params()
        self.runner.start(sig, params, ix, iv)
        self.btn_run.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.lbl_status.configure(text="● RUNNING", text_color=ACCENT3)

    def _stop_sim(self):
        self.runner.stop()
        self.btn_run.configure(state="normal")
        self.btn_stop.configure(state="disabled")
        self.lbl_status.configure(text="● STOPPED", text_color=ACCENT2)

    # ── Periodic UI refresh ───────────────────────────────────────────────────
    def _ui_update_loop(self):
        self._update_metrics()
        self.lbl_simx.configure(text=f"{self.runner.current_x:+.4f}")
        self.lbl_time.configure(text=f"t = {self.system.t:.3f} s")

        if len(self.runner.hist_t) >= 2:
            t  = self.runner.hist_t
            self.line_x.set_data(t, self.runner.hist_x)
            self.ax_x.relim(); self.ax_x.autoscale_view()
            self.ax_x.set_xlim(t[0], max(t[-1], t[0] + 1))

            self.line_f.set_data(t, self.runner.hist_F)
            self.ax_f.relim(); self.ax_f.autoscale_view()
            self.ax_f.set_xlim(t[0], max(t[-1], t[0] + 1))

            self.canvas_mpl.draw_idle()

        if self.runner.finished:
            self.btn_run.configure(state="normal")
            self.btn_stop.configure(state="disabled")
            self.lbl_status.configure(text="● SETTLED", text_color=ACCENT3)
            self.runner.finished = False

        self.after(50, self._ui_update_loop)

    def on_close(self):
        self.runner.stop()
        self.link.disconnect()
        self.destroy()


# =============================================================================
if __name__ == "__main__":
    app = SuspenSimApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()