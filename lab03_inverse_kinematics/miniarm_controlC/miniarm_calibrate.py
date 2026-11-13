"""
miniarm_calibrate.py  -  enter your arm's calibration and save calibration.json
==============================================================================

This program does not talk to the arm. Run it next to miniarm_controlC.py:

  1. In miniarm_controlC.py, use APP CONTROL to move ONE joint at a time.
  2. Measure that joint's angle relative to the previous link on the polar paper:
       90 = in line with the previous link; bending forward (base: turning left) = larger.
  3. Type the servo value (from the controlC "Servo values" readout) and the angle here.
     Five points per joint, spread across its range.
  4. Fill in each joint's safe range and the claw's open/closed values.
  5. Click Save, then "Reload calibration" in miniarm_controlC.py.

The file is saved next to this program as calibration.json.
"""

import os
import tkinter as tk
from tkinter import scrolledtext

import miniarm_calibration as mc

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
ROWS = mc.RECOMMENDED_POINTS

JOINT_HELP = {
    "base":    "relative to straight ahead: 90 = ahead, larger = to the arm's left",
    "arm":     "relative to the base post: 90 = straight up, larger = forward",
    "forearm": "relative to the arm link: 90 = in line, larger = bent forward",
    "wrist":   "relative to the forearm: 90 = in line, larger = bent forward",
}
PLOT_W, PLOT_H, PAD = 230, 170, 26


class CalibrateApp:
    def __init__(self, root):
        self.root = root
        root.title("miniArm Calibration")
        root.resizable(False, False)
        self.entries = {}       # joint -> list of (servo Entry, angle Entry)
        self.ranges = {}        # joint or "claw" -> (min Entry, max Entry)
        self.plots = {}

        tk.Label(root, justify="left", font=("Segoe UI", 9), padx=12, pady=8, text=(
            "Move one joint at a time with APP CONTROL in miniarm_controlC.py, measure its angle on the polar "
            "paper relative to the previous link (90 = in line), and type the servo value and angle below.\n"
            "Five points per joint, spread across its range. Then Save, and click \"Reload calibration\" "
            "in miniarm_controlC.py.")).pack(anchor="w")

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
        frame = tk.LabelFrame(parent, text=f"  {name.upper()}  ({JOINT_HELP[name]})  ",
                              font=("Segoe UI", 9, "bold"), padx=8, pady=6)
        table = tk.Frame(frame)
        table.grid(row=0, column=0, sticky="n")
        tk.Label(table, text="Point", font=("Segoe UI", 8)).grid(row=0, column=0)
        tk.Label(table, text="Servo", font=("Segoe UI", 8)).grid(row=0, column=1)
        tk.Label(table, text="Angle (deg)", font=("Segoe UI", 8)).grid(row=0, column=2)
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
        tk.Label(rng, text="Safe range (shaded)", font=("Segoe UI", 8)).pack(side="left")
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
                return float(text)
            except ValueError:
                problems.append(f"{label}: '{text}' is not a number")
                return None

        data = {"joints": {}, "claw": {}}
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
            for name in mc.JOINTS:                     # sensible starting safe ranges
                for e, v in zip(self.ranges[name], (0, 180)):
                    e.delete(0, "end")
                    e.insert(0, str(v))
                self.draw(name)
            self.say("No calibration.json yet: fill in the form and Save.", clear=True)
            return
        try:
            data = mc.Calibration.load(CALIBRATION_FILE).data
        except mc.CalibrationError as e:
            self.say(str(e), "error", clear=True)
            return
        self.fill_form(data)
        for name in mc.JOINTS:
            self.draw(name)
        self.say(f"Loaded {CALIBRATION_FILE}.", "ok", clear=True)

    @staticmethod
    def _as_ints(data):
        """Servo values are whole numbers; angles may have decimals."""
        out = {"joints": {}, "claw": {k: (int(round(v)) if v is not None else None)
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
        """Servo value (across) vs measured angle (up), with the safe range shaded."""
        c = self.plots[name]
        c.delete("all")
        x = lambda s: PAD + s / 180 * (PLOT_W - PAD - 8)
        y = lambda a: PLOT_H - PAD + 4 - a / 180 * (PLOT_H - PAD - 4)

        lo, hi = self.ranges[name]
        try:
            c.create_rectangle(x(float(lo.get())), y(180), x(float(hi.get())), y(0),
                               fill="#e7f5ff", outline="")
        except ValueError:
            pass
        for v in (0, 90, 180):
            c.create_line(x(0), y(v), x(180), y(v), fill="#adb5bd" if v == 90 else "#e9ecef",
                          dash=(3, 3) if v == 90 else ())
            c.create_text(PAD - 4, y(v), text=str(v), anchor="e", font=("Segoe UI", 7), fill="#868e96")
        for v in (0, 90, 180):
            c.create_text(x(v), PLOT_H - 8, text=str(v), font=("Segoe UI", 7), fill="#868e96")
        c.create_text(x(45), PLOT_H - 8, text="servo value", font=("Segoe UI", 7), fill="#868e96")

        pts = []
        for s_e, a_e in self.entries[name]:
            try:
                pts.append((float(s_e.get()), float(a_e.get())))
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
