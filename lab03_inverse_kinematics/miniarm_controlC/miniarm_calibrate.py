"""
miniarm_calibrate.py  -  enter your arm's calibration and save calibration.json
==============================================================================

This program does not talk to the arm. Run it next to miniarm_controlC.py:

  1. In miniarm_controlC.py, use APP CONTROL to move ONE joint at a time.
  2. Measure the DH joint angle q using a protractor or a signed-angle tool.
       Base 90 forward; shoulder 90 upright; elbow/wrist 0 when aligned.
  3. Type the APPLIED raw servo command and measured q here, in degrees.
     Five points per joint, spread across its range.
  4. Fill in each joint's safe range and the claw's open/closed values.
  5. Click Save, then "Reload calibration" in miniarm_controlC.py.

The file is saved next to this program as calibration.json.
"""

import os
import math
import tkinter as tk
from tkinter import scrolledtext

import miniarm_calibration as mc

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
ROWS = mc.RECOMMENDED_POINTS

JOINT_HELP = {
    "base":    "BASE / S5: q = 0 right, 90 forward, 180 left",
    "arm":     "SHOULDER / S4: q = 0 horizontal forward, 90 upright",
    "forearm": "ELBOW / S3: q = 0 aligned; negative bends down",
    "wrist":   "WRIST / S2: q = 0 aligned; negative bends down",
}
PLOT_W, PLOT_H, PAD = 250, 190, 34

MEASUREMENT_HELP = '''WHY CALIBRATE?
The 3D target is XYZ in millimetres. IK calculates four DH joint angles q.
The servos accept raw command values. A raw value of 90 does not guarantee
a physical joint angle of 90 degrees: mounting, direction and scale differ.
Calibration records which raw command produces which measured joint angle.

TARGET XYZ -> 3D IK -> JOINT ANGLES q -> CALIBRATION -> RAW SERVO COMMANDS

WHAT TO ENTER
Each row is ONE pair: (applied raw command, measured joint q in degrees).
These are not (radius, angle) target coordinates and not XYZ measurements.
The plots are ordinary command-versus-joint-angle graphs, not workspace maps.
Printed polar paper is not required. A protractor is simply an angle-measuring tool.

JOINT REFERENCES (view from behind at the Arduino/knobs)
Base/S5: 0 right, 90 forward, 180 left; measure from above.
Shoulder/S4: 0 horizontal forward, 90 vertically up; measure from the side.
Elbow/S3: 0 when forearm continues along the upper-arm direction.
Wrist/S2: 0 when the tool continues along the forearm direction.
Elbow/wrist down-forward bending is negative; up-backward bending is positive.
Align a protractor with the preceding link's FORWARD EXTENSION, not the ray
pointing back to the preceding joint. See ../CALIBRATION_GUIDE.md for sketches.

ONE PAIR AT A TIME
1. Read the startup instructions and initialize with instructor guidance.
2. Select APP CONTROL in the controller; keep other joint sliders unchanged.
3. Set this joint's slider, click Send manual sliders, and wait for completion.
4. Read its applied raw command and independently measure the physical q.
5. Enter that pair here. Moving a slider without sending is not a measurement.
Start with two pairs in an instructor demonstration. For the lab collect five
spread across the usable range, without driving into mechanical stops.

EXAMPLE ONLY - NOT SETTINGS FOR YOUR ARM
Raw commands 60, 80, 100, 120, 140 might measure wrist q = 30, 15, 0, -15, -30.
Then q = -22.5 degrees corresponds to raw command 130 by interpolation.
Your arm's measurements will differ. Never copy example numbers as calibration.

SAFE RANGES AND CLAW
Safe min/max are RAW commands for your measured usable range, not q angles.
Do not assume the entire controller range 0..180 is mechanically usable.
S1 claw open/closed values are raw commands; claw opening is separate from wrist q.

SAVE AND USE
Click Check, resolve errors, Save, then Reload calibration in the controller.
Reloading does not move the arm; it requires explicit initialization again.
New files declare angle_convention=joint. Old files are converted for display
and remain readable by the updated controller. Update the controller, calibration
module and this form together. Use XYZ/rulers for target-position validation.
'''


def angle_ticks(name):
    return (0,90,180) if name in ('base','arm') else (-180,-90,0,90,180)


class CalibrateApp:
    def __init__(self, root):
        self.root = root
        root.title("miniArm Joint-angle Calibration for XYZ IK")
        root.resizable(False, False)
        self.entries = {}       # joint -> list of (servo Entry, angle Entry)
        self.ranges = {}        # joint or "claw" -> (min Entry, max Entry)
        self.plots = {}

        tk.Label(root, justify="left", font=("Segoe UI", 9), padx=12, pady=8, wraplength=1000, text=(
            "XYZ targets -> IK joint angles q -> calibrated raw servo commands. This form measures JOINT angles; enter XYZ in the controller.\n"
            "In APP CONTROL, send one joint's command, wait, measure its DH angle q, then record the pair. Elbow/wrist: 0 = aligned; down = negative.\n"
            "Use a protractor or signed-angle tool; polar paper is not required. Read Why / How first. Collect five pairs per joint, then Check and Save.")).pack(anchor="w")
        tk.Button(root,text='Why calibrate? / How to measure',command=self.show_help).pack(anchor='w',padx=12,pady=(0,4))

        grid = tk.Frame(root, padx=8)
        grid.pack()
        for n, name in enumerate(mc.JOINTS):
            self._build_joint(grid, name).grid(row=n // 2, column=n % 2, padx=6, pady=6, sticky="n")

        self._build_claw_and_buttons()
        self.messages = scrolledtext.ScrolledText(root, width=110, height=7, font=("Consolas", 9),
                                                  state="disabled", wrap="word")
        self.messages.pack(padx=12, pady=(0, 12))
        for tag, color in (("error", "#e03131"), ("warn", "#e8590c"), ("ok", "#2b8a3e")):
            self.messages.tag_config(tag, foreground=color)

        self.load()

    # ------------------------------------------------------------------ layout
    def _build_joint(self, parent, name):
        frame = tk.LabelFrame(parent, text=f"  {JOINT_HELP[name]}  ",
                              font=("Segoe UI", 9, "bold"), padx=8, pady=6)
        table = tk.Frame(frame)
        table.grid(row=0, column=0, sticky="n")
        tk.Label(table, text="Point", font=("Segoe UI", 8)).grid(row=0, column=0)
        tk.Label(table, text="Raw command", font=("Segoe UI", 8)).grid(row=0, column=1)
        tk.Label(table, text="Joint q (deg)", font=("Segoe UI", 8)).grid(row=0, column=2)
        rows = []
        for r in range(ROWS):
            tk.Label(table, text=str(r + 1), font=("Segoe UI", 8)).grid(row=r + 1, column=0)
            s = tk.Entry(table, width=6, justify="center")
            a = tk.Entry(table, width=8, justify="center")
            s.grid(row=r + 1, column=1, padx=2, pady=1)
            a.grid(row=r + 1, column=2, padx=2, pady=1)
            for e in (s, a):
                e.bind("<KeyRelease>", lambda ev, n=name: self.draw(n))
            rows.append((s, a))
        self.entries[name] = rows

        rng = tk.Frame(table)
        rng.grid(row=ROWS + 1, column=0, columnspan=3, pady=(8, 0))
        tk.Label(rng, text="Safe RAW range", font=("Segoe UI", 8)).pack(side="left")
        lo = tk.Entry(rng, width=5, justify="center")
        hi = tk.Entry(rng, width=5, justify="center")
        lo.pack(side="left", padx=2)
        tk.Label(rng, text="to", font=("Segoe UI", 8)).pack(side="left")
        hi.pack(side="left", padx=2)
        for e in (lo, hi):
            e.bind("<KeyRelease>", lambda ev, n=name: self.draw(n))
        self.ranges[name] = (lo, hi)

        c = tk.Canvas(frame, width=PLOT_W, height=PLOT_H, bg="white", highlightthickness=1,
                      highlightbackground="#ced4da")
        c.grid(row=0, column=1, padx=(10, 0))
        self.plots[name] = c
        return frame

    def _build_claw_and_buttons(self):
        bar = tk.Frame(self.root, padx=14, pady=4)
        bar.pack(fill="x")
        claw = tk.LabelFrame(bar, text="  CLAW  ", font=("Segoe UI", 9, "bold"), padx=8, pady=6)
        claw.pack(side="left")
        self.claw = {}
        for col, (key, label) in enumerate([("open", "Open"), ("closed", "Closed"),
                                            ("min", "Safe min"), ("max", "Safe max")]):
            tk.Label(claw, text=label, font=("Segoe UI", 8)).grid(row=0, column=col, padx=6)
            e = tk.Entry(claw, width=6, justify="center")
            e.grid(row=1, column=col, padx=6)
            self.claw[key] = e

        buttons = tk.Frame(bar)
        buttons.pack(side="right")
        tk.Button(buttons, text="Check", width=10, command=self.check).pack(side="left", padx=4)
        tk.Button(buttons, text="Reload file", width=10, command=self.load).pack(side="left", padx=4)
        tk.Button(buttons, text="Save", width=10, bg="#2b8a3e", fg="white",
                  font=("Segoe UI", 9, "bold"), command=self.save).pack(side="left", padx=4)
        tk.Label(self.root, text=f"File: {CALIBRATION_FILE}", font=("Segoe UI", 8), fg="#666",
                 padx=14).pack(anchor="w")

    # ------------------------------------------------------------------ messages
    def show_help(self):
        window=tk.Toplevel(self.root);window.title('Why calibration is needed for XYZ IK')
        body=scrolledtext.ScrolledText(window,width=96,height=32,wrap='word',font=('Segoe UI',10))
        body.pack(fill='both',expand=True,padx=12,pady=12)
        body.insert('1.0',MEASUREMENT_HELP);body.config(state='disabled')

    def say(self, text, tag=None, clear=False):
        self.messages.config(state="normal")
        if clear:
            self.messages.delete("1.0", "end")
        self.messages.insert("end", text + "\n", tag)
        self.messages.see("end")
        self.messages.config(state="disabled")

    # ------------------------------------------------------------------ form <-> data
    def read_form(self):
        """Form -> calibration data dict, plus any problems with what was typed."""
        problems = []

        def number(entry, label):
            text = entry.get().strip()
            if not text:
                return None
            try:
                value=float(text)
                if not math.isfinite(value):raise ValueError
                return value
            except ValueError:
                problems.append(f"{label}: '{text}' must be a finite number")
                return None

        data = {"angle_convention": "joint", "joints": {}, "claw": {}}
        for name in mc.JOINTS:
            points = []
            for r, (s_e, a_e) in enumerate(self.entries[name]):
                s, a = number(s_e, f"{name} point {r + 1} servo"), number(a_e, f"{name} point {r + 1} angle")
                if (s is None) != (a is None):
                    problems.append(f"{name} point {r + 1}: needs both a servo value and an angle")
                elif s is not None:
                    points.append([s, a])
            lo, hi = (number(e, f"{name} safe range") for e in self.ranges[name])
            if lo is None or hi is None:
                problems.append(f"{name}: fill in the safe range")
            data["joints"][name] = {"points": points, "min": lo, "max": hi}
        for key, e in self.claw.items():
            v = number(e, f"claw {key}")
            if v is None:
                problems.append(f"claw: fill in '{key}'")
            data["claw"][key] = v
        return data, problems

    def fill_form(self, data):
        for name in mc.JOINTS:
            j = data["joints"][name]
            pts = sorted(j["points"])
            for r, (s_e, a_e) in enumerate(self.entries[name]):
                s_e.delete(0, "end")
                a_e.delete(0, "end")
                if r < len(pts):
                    s_e.insert(0, f"{pts[r][0]:g}")
                    a_e.insert(0, f"{pts[r][1]:g}")
            for e, v in zip(self.ranges[name], (j["min"], j["max"])):
                e.delete(0, "end")
                e.insert(0, f"{v:g}")
        for key, e in self.claw.items():
            e.delete(0, "end")
            e.insert(0, f"{data['claw'][key]:g}")

    # ------------------------------------------------------------------ actions
    def check(self):
        data, problems = self.read_form()
        errors, warnings = ([], []) if problems else mc.check(self._as_ints(data))
        errors = problems + errors
        self.say("Check:", clear=True)
        for e in errors:
            self.say("  ERROR   " + e, "error")
        for w in warnings:
            self.say("  warning " + w, "warn")
        if not errors:
            self.say("  OK: this calibration can be saved." + (" (See the warnings.)" if warnings else ""), "ok")
        for name in mc.JOINTS:
            self.draw(name)
        return data, errors

    def save(self):
        data, errors = self.check()
        if errors:
            self.say("Not saved: fix the errors first.", "error")
            return
        mc.Calibration.save(self._as_ints(data), CALIBRATION_FILE)
        self.say(f"Saved {CALIBRATION_FILE}. Click \"Reload calibration\" in miniarm_controlC.py.", "ok")

    def load(self):
        if not os.path.exists(CALIBRATION_FILE):
            for name in mc.JOINTS:
                self.draw(name)
            self.say("No calibration.json yet. Read Why / How. Enter measured q and instructor-reviewed RAW safe limits; 0..180 is not assumed safe.", clear=True)
            return
        try:
            loaded = mc.Calibration.load(CALIBRATION_FILE)
            data = loaded.as_joint_data()
        except mc.CalibrationError as e:
            self.say(str(e), "error", clear=True)
            return
        self.fill_form(data)
        for name in mc.JOINTS:
            self.draw(name)
        self.say(f"Loaded {CALIBRATION_FILE}.", "ok", clear=True)
        if loaded.angle_convention=='legacy':
            self.say('Older 90-reference table converted to DH q for display. Save writes angle_convention=joint with the same command mapping.', 'warn')

    @staticmethod
    def _as_ints(data):
        """Servo values are whole numbers; angles may have decimals."""
        out = {"angle_convention": data.get('angle_convention','legacy'), "joints": {}, "claw": {k: (int(round(v)) if v is not None else None)
                                     for k, v in data["claw"].items()}}
        for name, j in data["joints"].items():
            out["joints"][name] = {
                "points": [[int(round(s)), a] for s, a in j["points"]],
                "min": int(round(j["min"])) if j["min"] is not None else None,
                "max": int(round(j["max"])) if j["max"] is not None else None,
            }
        return out

    # ------------------------------------------------------------------ plot
    def draw(self, name):
        """Raw command versus DH joint q; this is not a polar target plot."""
        c = self.plots[name]
        c.delete("all")
        ticks=angle_ticks(name);amin,amax=ticks[0],ticks[-1]
        x = lambda s: 40 + s / 180 * (PLOT_W - 48)
        y = lambda a: PLOT_H - 42 - (a-amin)/(amax-amin) * (PLOT_H - 68)
        c.create_text(PLOT_W/2,10,text='Measured joint q (deg)',font=('Segoe UI',7),fill='#495057')

        lo, hi = self.ranges[name]
        try:
            limits=[float(lo.get()),float(hi.get())]
            if not all(math.isfinite(v) for v in limits):raise ValueError
            c.create_rectangle(x(limits[0]), y(amax), x(limits[1]), y(amin),
                               fill="#e7f5ff", outline="")
        except ValueError:
            pass
        for v in ticks:
            reference=mc.REFERENCE_Q[name]
            c.create_line(x(0), y(v), x(180), y(v), fill="#adb5bd" if v == reference else "#e9ecef",
                          dash=(3, 3) if v == reference else ())
            c.create_text(35, y(v), text=str(v), anchor="e", font=("Segoe UI", 7), fill="#868e96")
        for v in (0, 90, 180):
            c.create_text(x(v), PLOT_H - 27, text=str(v), font=("Segoe UI", 7), fill="#868e96")
        c.create_text(PLOT_W/2, PLOT_H - 10, text="Raw servo command", font=("Segoe UI", 7), fill="#868e96")

        pts = []
        for s_e, a_e in self.entries[name]:
            try:
                pair=(float(s_e.get()), float(a_e.get()))
                if all(math.isfinite(v) for v in pair):pts.append(pair)
            except ValueError:
                pass
        pts.sort()
        for (s0, a0), (s1, a1) in zip(pts, pts[1:]):
            c.create_line(x(s0), y(a0), x(s1), y(a1), fill="#1c7ed6", width=2)
        for s, a in pts:
            c.create_oval(x(s) - 3, y(a) - 3, x(s) + 3, y(a) + 3, fill="#1c7ed6", outline="")


if __name__ == "__main__":
    root = tk.Tk()
    CalibrateApp(root)
    root.mainloop()

