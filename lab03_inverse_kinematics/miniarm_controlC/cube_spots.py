"""cube_spots.py - random, reachable cube spots for the pick & place contest.

Put this file in the miniarm_controlC folder (next to miniarm_controlC.py and
calibration.json) and run:   python cube_spots.py

Enter a seed and the number of cubes, choose the Z option, press CALCULATE.
Same seed + same inputs = same layout.

Every spot is checked with the project's own pick & place planner: the COMPLETE
task from the upright pose (open, approach, lower, grip, lift, carry to the cup,
release, retreat) must plan, with the same IK, servo-limit, collision and cup
checks as miniarm_controlC.py, using GEOMETRY from miniarm_controlC.py and your
calibration.json. Nothing is sent to the arm.

Only two cubes are on the table at once, so only cubes that are next to each
other in the list (1&2, 2&3, ...) are kept apart.
"""
import ast
import math
import queue
import random
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import miniarm_calibration as mc   # noqa: E402
import miniarm_ik as ik            # noqa: E402
import miniarm_pick_place as pp    # noqa: E402

# =============================================================================
# SETTINGS (mm). World frame: X right, Y forward, Z up, origin under the base axis.
# =============================================================================
DEFAULT_SEED = 42
DEFAULT_CUBES = 5
CUBE_SIZE = 35.0              # cube edge length
STAND_SIZE = 60.0             # edge length of the (square) stand under a raised cube
SPACING_MM = 50.0             # min gap rule between consecutive cubes, centre to centre
                              # (grows automatically when a stand is involved)

CUP_X, CUP_Y = 125.0, 0.0     # cup centre on the table
CUP_RIM_Z = 125.0             # height of the cup rim
CUP_DIAMETER = 101.6          # cup opening diameter
CUP_CLEARANCE = 10.0          # extra gap between the cup wall and a cube/stand

BASE_RADIUS = 55.0            # keep cubes/stands outside this circle around the base axis

SEARCH_X = (-220.0, 220.0)    # random cube centres are drawn from this box
SEARCH_Y = (0.0, 220.0)
GRID_MM = 5                   # X and Y are multiples of this (1 = any whole mm)
MAX_TRIES_PER_CUBE = 300      # random candidates per cube before giving up
MAX_SURFACE_Z = 200.0         # largest surface height accepted in the Z box
MOVE_SECONDS = 2.0            # same default move time as the controller
CALIBRATION_FILE = HERE / 'calibration.json'
# =============================================================================

CUBE_R = CUBE_SIZE / math.sqrt(2)      # footprint radius of a cube (any rotation)
STAND_R = STAND_SIZE / math.sqrt(2)    # footprint radius of a stand
HALF = CUBE_SIZE / 2                   # grip point = cube centre = surface + HALF


def footprint(surface_z):
    """Radius the cube (and its stand, if raised) takes up on the table."""
    return max(CUBE_R, STAND_R) if surface_z > 0 else CUBE_R


def load_geometry():
    """GEOMETRY exactly as written in miniarm_controlC.py (read, not imported)."""
    tree = ast.parse((HERE / 'miniarm_controlC.py').read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'GEOMETRY' for t in node.targets):
            return ast.literal_eval(node.value)
    raise ValueError('GEOMETRY not found in miniarm_controlC.py')


def setup_arm():
    if not CALIBRATION_FILE.exists():
        raise ValueError(f'No calibration file at {CALIBRATION_FILE}. Calibrate the arm first.')
    cal = mc.Calibration.load(CALIBRATION_FILE)
    m = ik.model(load_geometry(), cal)
    q = np.deg2rad([90., 90., 0., 0.])
    if hasattr(ik, 'model_angles'):          # builds with the L3_OFFSET wrist tilt
        q = ik.model_angles(q)
    start = ik.q_to_servo(cal, q, cal.claw_open)
    settings = pp.CupSettings(x=CUP_X, y=CUP_Y, rim_z=CUP_RIM_Z, diameter=CUP_DIAMETER,
                              object_radius=max(5.0, CUBE_R)).validate()
    return m, cal, start, settings


def table_ok(x, y, surface_z, previous):
    """Cheap geometric checks before running the planner."""
    r = footprint(surface_z)
    if y - r <= 0:                                                  # in front of the base
        return False
    if math.hypot(x, y) < BASE_RADIUS + r:                          # clear of the base
        return False
    if math.hypot(x - CUP_X, y - CUP_Y) < CUP_DIAMETER / 2 + CUP_CLEARANCE + r:
        return False                                                # clear of the cup
    if previous:                                                    # clear of the cube before
        px, py, pz = previous
        extra = (r - CUBE_R) + (footprint(pz) - CUBE_R)
        if math.hypot(x - px, y - py) < SPACING_MM + extra:
            return False
    return True


def generate(seed, count, surfaces, progress=lambda text: None):
    """Return [(x, y, grip_z), ...]; raises ValueError with a readable message."""
    rng = random.Random(seed)
    heights = list(surfaces) + [0.0] * (count - len(surfaces))
    rng.shuffle(heights)                    # each typed Z used once, at a random cube
    m, cal, start, settings = setup_arm()
    spots, previous = [], None
    for i, surface in enumerate(heights, 1):
        grip_z = surface + HALF
        for _ in range(MAX_TRIES_PER_CUBE):
            x = GRID_MM * round(rng.uniform(*SEARCH_X) / GRID_MM)
            y = GRID_MM * round(rng.uniform(*SEARCH_Y) / GRID_MM)
            if not table_ok(x, y, surface, previous):
                continue
            progress(f'Planning cube {i}/{count}...')
            try:
                pp.build_plan(m, cal, start, np.array([x, y, grip_z]), settings, MOVE_SECONDS)
            except ValueError:
                continue
            spots.append((x, y, grip_z))
            previous = (x, y, surface)
            break
        else:
            raise ValueError(f'Cube {i} (surface z {surface:g} mm): no reachable spot found in '
                             f'{MAX_TRIES_PER_CUBE} tries. Try another seed or a lower Z.')
    return spots


class App:
    def __init__(self, root):
        self.root = root
        root.title('Cube spots')
        root.minsize(560, 420)
        self.jobs = queue.Queue()
        self.running = False
        pad = dict(padx=8, pady=6)

        top = tk.Frame(root); top.pack(fill='x', **pad)
        tk.Label(top, text='Seed', font=('Segoe UI', 11)).pack(side='left')
        self.seed = tk.Entry(top, width=10, font=('Segoe UI', 11))
        self.seed.insert(0, str(DEFAULT_SEED)); self.seed.pack(side='left', padx=(6, 24))
        tk.Label(top, text='Number of cubes:', font=('Segoe UI', 11)).pack(side='left')
        self.count = tk.Entry(top, width=6, font=('Segoe UI', 11))
        self.count.insert(0, str(DEFAULT_CUBES)); self.count.pack(side='left', padx=6)

        zrow = tk.Frame(root); zrow.pack(fill='x', **pad)
        tk.Label(zrow, text='Z-axis?', font=('Segoe UI', 11)).pack(side='left')
        self.zmode = tk.StringVar(value='none')
        tk.Radiobutton(zrow, text=f'None ({HALF:g} mm)', variable=self.zmode, value='none',
                       font=('Segoe UI', 11), command=self.z_changed).pack(side='left', padx=(10, 16))
        tk.Radiobutton(zrow, variable=self.zmode, value='list',
                       command=self.z_changed).pack(side='left')
        self.zvalues = tk.Entry(zrow, width=34, font=('Segoe UI', 10), fg='#888')
        self.zvalues.insert(0, 'Z-values, separated by comma')
        self.zvalues.bind('<FocusIn>', self.z_focus)
        self.zvalues.pack(side='left', fill='x', expand=True)
        tk.Label(root, text='Z values = height of the surface (stand) a cube sits on, in mm. '
                            'Each is used once, for a random cube; the rest sit on the table.',
                 fg='#555', wraplength=540, justify='left').pack(fill='x', padx=8)

        self.button = tk.Button(root, text='CALCULATE', width=28, height=2,
                                command=self.calculate)
        self.button.pack(pady=10)
        self.status = tk.StringVar(value='')
        tk.Label(root, textvariable=self.status, fg='#555').pack()

        self.out = tk.Text(root, height=10, font=('Consolas', 11), relief='solid', bd=2)
        self.out.pack(fill='both', expand=True, **pad)
        self.out.config(state='disabled')
        root.after(100, self.poll)

    def z_focus(self, _event):
        if self.zvalues.cget('fg') == '#888':
            self.zvalues.delete(0, 'end'); self.zvalues.config(fg='black')
        self.zmode.set('list')

    def z_changed(self):
        if self.zmode.get() == 'list':
            self.zvalues.focus_set()

    def show(self, text):
        self.out.config(state='normal'); self.out.delete('1.0', 'end')
        self.out.insert('1.0', text); self.out.config(state='disabled')

    def read_inputs(self):
        text = self.seed.get().strip()
        if not text:
            raise ValueError('Enter a seed (any whole number).')
        try: seed = int(text)
        except ValueError: raise ValueError('Seed must be a whole number.')
        try: count = int(self.count.get().strip())
        except ValueError: raise ValueError('Number of cubes must be a whole number.')
        if not 1 <= count <= 50:
            raise ValueError('Number of cubes must be 1..50.')
        surfaces = []
        if self.zmode.get() == 'list' and self.zvalues.cget('fg') != '#888':
            for part in self.zvalues.get().split(','):
                if not part.strip(): continue
                try: z = float(part)
                except ValueError: raise ValueError(f'"{part.strip()}" is not a number.')
                if not 0 <= z <= MAX_SURFACE_Z:
                    raise ValueError(f'Z values must be 0..{MAX_SURFACE_Z:g} mm.')
                surfaces.append(z)
            if len(surfaces) > count:
                raise ValueError(f'{len(surfaces)} Z values but only {count} cubes.')
        return seed, count, surfaces

    def calculate(self):
        if self.running: return
        try:
            seed, count, surfaces = self.read_inputs()
        except ValueError as e:
            self.show('Error: ' + str(e)); return
        self.running = True; self.button.config(state='disabled')
        self.show(''); self.status.set('Starting...')

        def work():
            try:
                spots = generate(seed, count, surfaces,
                                 progress=lambda t: self.jobs.put(('progress', t)))
                self.jobs.put(('done', (seed, spots)))
            except Exception as e:      # shown in the window, never a silent crash
                self.jobs.put(('error', str(e)))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        try:
            while True:
                kind, value = self.jobs.get_nowait()
                if kind == 'progress':
                    self.status.set(value); continue
                self.running = False; self.button.config(state='normal')
                if kind == 'error':
                    self.status.set(''); self.show('Error: ' + value)
                else:
                    seed, spots = value
                    lines = [f'Cube {i}: [{x:4.0f}, {y:4.0f}, {z:5.1f}] xyz'
                             for i, (x, y, z) in enumerate(spots, 1)]
                    self.show('\n'.join(lines))
                    self.status.set(f'Seed {seed}: every spot passed a full pick & place plan to '
                                    f'the cup. Z = grip point (cube centre).')
        except queue.Empty:
            pass
        self.root.after(100, self.poll)


if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
