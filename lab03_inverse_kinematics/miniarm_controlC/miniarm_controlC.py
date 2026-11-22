"""miniArm XYZ GUI. Run --simulate for offline use or --port COM11 for hardware.

Keep this folder next to ../python. Core uses metres/radians; entries use mm.
APP and KNOB controls are manual and bypass IK collision checks.
"""
SERIAL_PORT = "COM11"      # Arduino IDE: Tools -> Port shows the right one
BAUD_RATE = 115200         # must match miniarm_controlC.ino

# --- Calibration ---
# Servo ranges, joint angles, middle pose and claw values come from this file,
# written by miniarm_calibrate.py and saved next to this program.
CALIBRATION_FILE = "calibration.json"

# --- Arm dimensions in mm (measure axis to axis) ---
GEOMETRY = {
    "H": 120.0,   # table surface to the center of the arm (S4) shaft
    "L1": 56.5,   # arm axis to forearm axis
    "L2": 61.5,   # forearm axis to wrist axis
    "L3": 90.0,  # wrist axis to the grip point (middle of the claw fingers), along the claw
    "L3_OFFSET": 10.0,  # grip point sideways from the claw line: + = claw's top side
                        # (+Z when the claw points forward level). Handled as a fixed
                        # wrist tilt of atan(L3_OFFSET/L3); upright stays fully
                        # straight. 0 = original behaviour.
}

# --- Behaviour ---
STEP_MS = 15               # ms per servo unit while moving (bigger = slower, smoother)
SETTLE_MS = 0            # IK CONTROL moves one joint at a time; pause between joints (ms)
KNOB_TAKEOVER_DEG = 10     # turning a knob this far switches to KNOB CONTROL
UPRIGHT_MIN_SECONDS = 5.0  # gentle upright return after initialization
UPRIGHT_MAX_SERVO_SPEED = 15.0  # peak raw servo-command units/second (not measured joint speed)
STARTUP_SERVO_PAUSE_MS = 1000  # firmware SINIT: pause after each first enable (1000..5000)
MAX_MISS_MM = 10           # maximum predicted XYZ miss after calibration/rounding
RETRY_SEEDS = True         # if IK fails from the arm's current pose, retry from other starting poses

# --- Window ---
WINDOW_SIZE = "1400x850"   # starting size; the window can be resized
NUDGE = 15                 # APP CONTROL +/- buttons change a servo value by this much
# --- Overshoot-and-return finish ---
# After every smooth move, joints listed here step a little PAST their target
# (continuing the direction they were moving) and then back. The servo then
# makes a firm final correction instead of creeping to a stop inside its dead
# zone short of the target. Set FINISH_OVERSHOOT to 0 to turn it off.
FINISH_OVERSHOOT = 6       # raw servo units past the target
FINISH_JOINTS = ("shoulder","elbow","wrist")   # any of "base", "shoulder", "elbow", "wrist"
FINISH_MIN_MOVE = 1        # skip joints that moved less than this many raw units
AUTO_PREVIEW_MS = 0      # IK: recalculate the preview this long after you stop typing (0 = off)

# =============================================================================
# END OF SETUP
# =============================================================================

import numpy as np
import math
import copy
import json
import os
import queue
import shutil
import tempfile
import threading
import time
import tkinter as tk
from tkinter import scrolledtext

import miniarm_calibration as mc
import miniarm_pick_place as pp
from miniarm_version import APP_VERSION, DEVELOPER
APP_VERSION = '2.2.1'  # adjustable calibrated claw Open/Closed presets

try:
    import miniarm_ik as ik                 # needs numpy
except ModuleNotFoundError as missing:
    raise SystemExit(f"{missing}. Install it with:  python -m pip install numpy pyserial")

try:
    import serial
except ImportError:            # hardware is unavailable; explicit --simulate still works
    serial = None

# Servo indexes, in read_mini.ino order
CLAW, WRIST, FOREARM, ARM, BASE = 0, 1, 2, 3, 4
JOINT_NAMES = ["Claw", "Wrist", "Elbow", "Shoulder", "Base"]
DISPLAY_ORDER = [BASE, ARM, FOREARM, WRIST, CLAW]   # bottom of the arm first
APP_ORDER = [BASE, ARM, FOREARM, WRIST, CLAW]       # APP CONTROL sliders, left to right
KNOB_GRID = [CLAW, WRIST, FOREARM, ARM, BASE, None] # KNOB CONTROL grid, 2 rows of 3; None = the unused 6th knob

MODES = {   # code sent to the Arduino: (button text, color)
    "A": ("APP CONTROL", "#2b8a3e"),
    "K": ("KNOB CONTROL", "#e8590c"),
    "I": ("IK CONTROL", "#1c7ed6"),
}

COLORS = {
    'status': '#f2c94c',    # status box outline (yellow)
    'app': '#2b8a3e', 'knob': '#e8590c', 'ik': '#1c7ed6',
    'send': '#d3f9d8',      # send buttons
    'pending': '#b2f2bb',   # value box: changed but not sent yet
    'arm': '#4b5563',       # 3D arm drawing outline (dark gray)
    'console': '#4b5563',   # console outline (dark gray)
    'error': '#7048a5',     # convergence plot outline (purple)
    'upright': '#b07a4f',   # upright button outline (brown)
}

# Camera locations in the SAME world frame as FK: X right, Y forward, Z up.
# Rear means the Arduino/knob side (negative Y). These are views, not new frames.
PREVIEW_CAMERAS = {
    'Rear right (Arduino side)': (1., -1., .8),
    'Rear (Arduino side)': (0., -1., .7),
    'Top': (0., 0., 1.),
    'Right side': (1., 0., 0.),
}


def preview_workspace_points(m,target=None):
    """Fixed workspace scale; independent of live and unsent servo values."""
    shoulder=ik.fk(m,np.zeros(len(m.mdh)))[1][1]*1000
    reach=max(70.,float(1000*(np.sum(np.abs(m.mdh[1:,0]))+
                  np.sum(np.abs(m.mdh[1:,2]))+np.linalg.norm(m.tool[:3,3]))))
    points=[shoulder+reach*np.array([x,y,z])
            for x in (-1,1) for y in (-1,1) for z in (-1,1)]
    points.extend([np.zeros(3),*list(np.eye(3)*70.)])
    if target is not None:points.append(np.asarray(target,dtype=float))
    return np.asarray(points)


def preview_camera_basis(view):
    """Orthonormal screen-right/up vectors and the camera's world direction.

    right cross up = camera. In particular the default camera is ABOVE the
    table; the former hard-coded projection had a negative camera Z component.
    Screen pixel Y increases down, so projection negates the up component.
    """
    camera=np.array(PREVIEW_CAMERAS[view],dtype=float)
    camera/=np.linalg.norm(camera)
    reference_up=np.array([0.,1.,0.]) if view=='Top' else np.array([0.,0.,1.])
    right=np.cross(reference_up,camera); right/=np.linalg.norm(right)
    up=np.cross(camera,right)
    return right,up,camera


def fit_preview_projection(points,view,viewport):
    """Fit world points (mm) into a canvas rectangle with ONE uniform scale."""
    right,up,_=preview_camera_basis(view)
    points=np.asarray(points,dtype=float)
    projected=np.column_stack((points@right,-points@up))
    low=projected.min(axis=0); high=projected.max(axis=0)
    x0,y0,x1,y1=viewport
    scale=min((x1-x0)/max(high[0]-low[0],1.),(y1-y0)/max(high[1]-low[1],1.))
    offset=np.array([(x0+x1)/2,(y0+y1)/2])-scale*(low+high)/2
    def project(p):
        p=np.asarray(p,dtype=float)
        return tuple(offset+scale*np.array([p@right,-p@up]))
    return project


# Extra IK starting poses (degrees: base, shoulder, elbow, wrist), tried only when
# the solve from the arm's current pose fails. None = base aimed at the target.
RETRY_POSES = [
    (90., 90., 0., 0.),          # upright
    (None, 60., -60., -60.),     # MATLAB's demo start, base aimed at the target
    (None, 90., -45., -45.),     # high and bent
    (None, 30., -60., -30.),     # low and reaching
    (90., 60., -60., -60.),
]


def retry_seeds(m, target_mm):
    """Starting poses for retries, aimed at the target and kept inside joint limits."""
    aim = math.degrees(math.atan2(target_mm[1], target_mm[0]))   # base 0 right, 90 forward
    seeds = []
    for pose in RETRY_POSES:
        q = np.deg2rad([aim if v is None else v for v in pose])
        q = np.clip(q, m.limits[:, 0], m.limits[:, 1])
        if not any(np.allclose(q, s) for s in seeds):
            seeds.append(q)
    return seeds


def calibration_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), CALIBRATION_FILE)


def claw_calibration(cal,opened,closed):
    """Copy only the two S1 presets; never expand measured safe limits."""
    if cal is None: raise ValueError('Load a measured calibration first')
    values=[]
    for label,value in (('Open',opened),('Closed',closed)):
        try: value=float(value)
        except (TypeError,ValueError): raise ValueError(label+' must be an integer raw servo value') from None
        if not math.isfinite(value) or value!=round(value):
            raise ValueError(label+' must be an integer raw servo value')
        low,high=cal.limits(CLAW)
        if not low<=value<=high: raise ValueError(f'{label} must stay within calibrated S1 range {low}..{high}')
        values.append(int(value))
    if values[0]==values[1]: raise ValueError('Open and Closed must be different values')
    data=copy.deepcopy(cal.data)
    data['claw']['open'],data['claw']['closed']=values
    errors,_=mc.check(data)
    if errors: raise ValueError('; '.join(errors))
    return mc.Calibration(data)


def save_claw_calibration(cal,opened,closed,path):
    """Preserve disk joint/metadata fields; backup and atomically replace JSON."""
    if cal is None: raise ValueError('Load a measured calibration first')
    disk=mc.Calibration.load(path)
    if (disk.data['joints']!=cal.data['joints'] or disk.angle_convention!=cal.angle_convention
            or disk.limits(CLAW)!=cal.limits(CLAW)):
        raise ValueError('Calibration changed on disk. Reload calibration before saving claw settings.')
    updated=claw_calibration(disk,opened,closed)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=os.path.dirname(os.path.abspath(path)),
                                         prefix='.claw_',suffix='.json',delete=False) as stream:
            temporary=stream.name
            json.dump(updated.data,stream,indent=2); stream.write('\n')
            stream.flush(); os.fsync(stream.fileno())
        shutil.copy2(path,str(path)+'.before_claw_edit')
        os.replace(temporary,path); temporary=None
    finally:
        if temporary is not None:
            try: os.unlink(temporary)
            except OSError: pass
    return updated


def load_startup_calibration(simulate, use_calibration=False):
    """Offline exercises are reproducible even if hardware calibration exists.

    --simulate --use-calibration explicitly tests the saved mapping and limits.
    Never replace an explicitly requested file with a synthetic mapping.
    Hardware without calibration keeps the supervised raw-neutral workflow.
    """
    if simulate and not use_calibration:
        return ik.demo_calibration(), True
    try:
        return mc.Calibration.load(calibration_path()), False
    except mc.CalibrationError:
        if simulate:
            raise  # explicit saved-calibration test must report invalid/missing data
        return None, False


# =============================================================================
# Connections: the real arm, or a simulated one
# =============================================================================
def clean_line(raw):
    """Bytes from the serial port -> a clean text line.

    When the port opens, the Uno resets and some boards' USB chips send a burst of
    garbled bytes first, which can land on the same line as "READY". Keep only what
    comes after the last garbled byte, and only plain printable characters.
    """
    text = raw.decode("ascii", errors="replace")
    text = text.rsplit("�", 1)[-1]                  # drop everything up to the last junk byte
    return "".join(ch for ch in text if " " <= ch <= "~").strip()


class SerialLink:
    """Talks to miniarm_controlC.ino. A background thread reads lines into a queue;
    the GUI reads the queue (Tkinter must only be touched from the main thread)."""

    simulated = False
    BOOT_TIMEOUT = 10.0
    OPEN_ATTEMPTS = 3
    RETRY_DELAY = 1.0

    def __init__(self, port, baud, inbox):
        self.inbox = inbox
        self.port = None
        self.port_name = port
        self.baud = baud
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._connected = False
        self._thread = threading.Thread(target=self._read_forever, daemon=True)
        self._thread.start()

    def _release_port(self):
        with self._lock:
            handle, self.port = self.port, None
            self._connected = False
        if handle is not None:
            try: handle.close()
            except Exception: pass

    def _read_forever(self):
        # Only retry before READY. A lost live connection requires an explicit
        # Retry click and a fresh initialization; never replay queued movement.
        for attempt in range(1, self.OPEN_ATTEMPTS + 1):
            if self._stop.is_set(): return
            ready = False
            try:
                self.inbox.put(f'LINK Opening {self.port_name} ({attempt}/{self.OPEN_ATTEMPTS})')
                handle = serial.Serial(self.port_name, self.baud, timeout=0.1,
                                       write_timeout=0.5)
                with self._lock:
                    if self._stop.is_set():
                        handle.close(); return
                    self.port = handle
                # Opening a USB Arduino usually resets it. A deliberate DTR
                # pulse also handles adapters that do not reset on open alone.
                # Never clear input here: FW/READY can already be arriving.
                try:
                    handle.dtr = False
                    if self._stop.wait(0.12): return
                    handle.dtr = True
                except (OSError, AttributeError) as reset_error:
                    self.inbox.put(f'LINK DTR reset unavailable; waiting for READY ({reset_error})')
                self.inbox.put(f'LINK Waiting for Arduino boot on {self.port_name}...')
                deadline = time.monotonic() + self.BOOT_TIMEOUT
                buffer = bytearray()
                firmware = None
                while not self._stop.is_set():
                    chunk = handle.read(max(1, min(handle.in_waiting, 4096)))
                    if chunk: buffer.extend(chunk)
                    while b'\n' in buffer:
                        raw, _, remainder = buffer.partition(b'\n')
                        buffer = bytearray(remainder)
                        line = clean_line(raw)
                        if not line: continue
                        if not ready:
                            if 'FW ' in line: firmware = line[line.rfind('FW '):]
                            if line.endswith('READY'):
                                # Publish the capabilities and READY together,
                                # only after a complete startup handshake.
                                if firmware: self.inbox.put(firmware)
                                with self._lock: self._connected = True
                                ready = True
                                self.inbox.put('READY')
                                self.inbox.put(f'LINK Connected to {self.port_name}')
                        else:
                            self.inbox.put(line)
                    if len(buffer) > 8192:
                        buffer = buffer[-1024:]  # bound noise, retain a possible READY suffix
                    if not ready and time.monotonic() > deadline:
                        raise TimeoutError('Port opened but no READY received. Check baud 115200, '
                                           'the miniArm control sketch, and USB connection; '
                                           'close Arduino Serial Monitor.')
                return
            except Exception as error:
                if self._stop.is_set(): return
                message = f'{self.port_name}: {error}'
                if ready:
                    self.inbox.put('LINK_ERROR Connection lost: ' + message)
                    return
                if attempt == self.OPEN_ATTEMPTS:
                    self.inbox.put('LINK_ERROR Could not connect: ' + message +
                                   ' Close Serial Monitor/other GUIs, check the port, then Retry connection.')
                else:
                    self.inbox.put('LINK Startup retry: ' + message)
            finally:
                self._release_port()
            if self._stop.wait(self.RETRY_DELAY): return

    def send(self, line):
        with self._lock:
            if self._stop.is_set() or not self._connected or self.port is None:
                raise ConnectionError('Arduino handshake is incomplete or the connection was lost')
            data = (line + '\n').encode('ascii')
            written = self.port.write(data)
            if written != len(data):
                raise ConnectionError('Incomplete serial write; command outcome is unknown')

    def tick(self):
        pass

    def close(self):
        self._stop.set()
        with self._lock: handle = self.port
        if handle is not None:
            try: handle.cancel_read()
            except (OSError, AttributeError): pass
        self._release_port()


class SimulatedArm:
    """Pretends to be miniarm_controlC.ino so the GUI works without hardware.
    Same commands and replies, except there are no knob readings."""

    simulated = True
    supports_timed = True
    firmware_version = '2.0.0'
    ORDER = [BASE, ARM, FOREARM, WRIST, CLAW]

    def __init__(self, inbox):
        self.inbox = inbox
        self.cur = [90] * 5
        self.tgt = [90] * 5
        self.limits = [(0, 180)] * 5
        self.on = False
        self.mode = "A"
        self.move_active = False
        self.seq = None            # [targets, index, settle_until]
        self.timed = None          # [start, end, start_time, duration_seconds]
        self.last_step = self.last_report = 0.0
        self.changed = False
        self.step_ms,self.settle_ms=STEP_MS,SETTLE_MS
        inbox.put("READY")

    def send(self, line):
        parts = line.split()
        if not parts:return
        cmd=parts[0]
        try:nums=[float(p) for p in parts[1:]]
        except ValueError:nums=[]
        clamp = lambda i, v: max(self.limits[i][0], min(self.limits[i][1], v))
        if cmd == "CFG" and len(nums) == 12 and all(np.isfinite(v) and v==int(v) for v in nums):
            self.limits = [(max(0,min(180,min(nums[2*i:2*i+2]))),
                            max(0,min(180,max(nums[2*i:2*i+2])))) for i in range(5)]
            self.step_ms,self.settle_ms=max(1,nums[10]),max(0,nums[11])
            self.inbox.put("CFG OK")
        elif cmd == "INIT" and len(nums) == 5 and np.isfinite(nums).all():
            self.timed=None
            self.seq,self.move_active=None,False  # INIT freezes old motion, as firmware does
            self.cur = [clamp(i, v) for i, v in enumerate(nums)]
            self.tgt = list(self.cur)
            self.on = self.changed = True
            self.inbox.put("OK")
        elif cmd == "T":
            if not self.on:self.inbox.put('ERR send INIT first');return
            if self.mode=='K':self.inbox.put('ERR in knob mode');return
            try:
                values=[float(p) for p in parts[1:]]
                if len(values)!=6 or not np.isfinite(values).all():raise ValueError
                duration=values[0];target=values[1:]
                if duration!=int(duration) or not 500<=duration<=25000:raise ValueError
                if any(not lo<=v<=hi for v,(lo,hi) in zip(target,self.limits)):raise ValueError
                if duration+1e-3<1500*max(abs(a-b) for a,b in zip(self.cur,target))/45:raise ValueError
            except ValueError:self.inbox.put('ERR invalid timed target/duration');return
            self.tgt=list(self.cur);self.seq=None;self.move_active=False
            self.timed=[list(self.cur),target,time.monotonic(),duration/1000]
        elif cmd == "MODE":
            if len(parts)!=2 or parts[1] not in ('A','K','I'):
                self.inbox.put('ERR MODE is A, K or I'); return
            self.tgt, self.move_active, self.seq = list(self.cur), False, None
            self.timed=None
            self.mode = parts[1]
            self.inbox.put(f"MODE {self.mode}")
        elif cmd in ("M", "J") and len(nums) == 5 and np.isfinite(nums).all():
            if not self.on:
                self.inbox.put('ERR send INIT first'); return
            if self.mode == "K":
                self.inbox.put("ERR in knob mode")
                return
            self.tgt, self.seq, self.move_active = list(self.cur), None, False
            self.timed=None
            targets = [clamp(i, v) for i, v in enumerate(nums)]
            if cmd == "M":
                self.tgt, self.move_active = targets, True
            else:
                self.seq = [targets, 0, 0.0]
        elif cmd == "STOP":
            self.tgt, self.move_active, self.seq = list(self.cur), False, None
            self.timed=None
            self.inbox.put("STOPPED")
            self.inbox.put("P " + " ".join(str(v) for v in self.cur))

    def _step(self, i, target):
        if self.cur[i] != target:
            delta=target-self.cur[i]
            self.cur[i]=target if abs(delta)<=1 else self.cur[i]+(1 if delta>0 else -1)
            self.changed = True
        return self.cur[i] == target

    def tick(self):
        now = time.monotonic()
        if self.on and self.timed is not None:
            start,end,began,duration=self.timed
            u=min(1.,max(0.,(now-began)/duration));fraction=u*u*(3-2*u)
            self.cur=[round(a+(b-a)*fraction,3) for a,b in zip(start,end)]
            self.tgt=list(self.cur);self.changed=True
            if u>=1:
                self.cur=list(end);self.tgt=list(end);self.timed=None;self.inbox.put('OK')
            self.last_step=now  # legacy ramps must not overwrite the timed curve
        while self.on and self.timed is None and now - self.last_step >= self.step_ms / 1000:
            self.last_step = now if now - self.last_step > 1 else self.last_step + self.step_ms / 1000
            if self.seq:
                targets, idx, settle_until = self.seq
                if now < settle_until:
                    break
                while idx < 5 and self.cur[self.ORDER[idx]] == targets[self.ORDER[idx]]:
                    idx += 1
                if idx >= 5:
                    self.seq = None
                    self.inbox.put("OK")
                    break
                j = self.ORDER[idx]
                self.tgt[j] = targets[j]
                arrived = self._step(j, targets[j])
                self.seq = [targets, idx, now + self.settle_ms / 1000 if arrived else 0.0]
            else:
                there = all([self._step(i, self.tgt[i]) for i in range(5)])
                if self.move_active and there:
                    self.move_active = False
                    self.inbox.put("OK")
        if self.changed and now - self.last_report >= 0.1:
            self.last_report, self.changed = now, False
            self.inbox.put("P " + " ".join(str(v) for v in self.cur))

    def close(self):
        pass


class ArmApp:
    def __init__(self, root, simulate=False, port='COM11', use_calibration=False):
        self.root=root
        self._closed=False; self._poll_id=None; self._auto_job=None
        self.simulate=simulate; self.hardware_port=port
        self._connecting=False; self.connection_state='Waiting for connection'
        root.title(f'miniArm XYZ Control v{APP_VERSION} — {"SIMULATION" if simulate else "REAL ROBOT"} — {DEVELOPER}')
        self.timed_supported=False;self.sequential_supported=False;self.firmware_version='unknown'
        self.inbox=queue.Queue(); self.link=None; self.ready=False
        self.mode='A'; self.positions=[90]*5; self.commanded=[90]*5
        self.initialized=False; self.busy=False; self.pending=None
        # servos_on: the Arduino has switched the servos on (INIT/SINIT finished).
        # It stays True after a calibration reload, an ERR or a timeout, which only
        # make the GUI's pose uncertain; it resets only when the Arduino resets.
        self.servos_on=False; self._recovering=False
        self.preview=None; self.plan=[]; self.active=None; self.started=0
        self._preview_plot=None
        self._app_target_pose=None
        self._app_target_points=None
        self._pp_settings=pp.CupSettings()
        self._pp_preview=None; self._pp_plot=None
        self._pp_running=False; self._pp_planning=False
        self._pp_delay=None; self._pp_steps=[]; self._pp_step_index=0
        self._pp_generation=0; self._pp_cancel=threading.Event()
        self._pp_messages=queue.Queue(); self._pp_current_label=''
        self._claw_dialog=None; self._claw_controls=None
        self.knobs=None; self.knob_baseline=None; self.synthetic=False
        self.stopped=False; self.stop_pending=False; self.stop_acknowledged=False; self.knob_takeover_pending=False
        self.cal,self.synthetic=load_startup_calibration(simulate,use_calibration)
        self.m=ik.model(GEOMETRY,self.cal)
        self._build()
        if simulate:
            self.link=SimulatedArm(self.inbox)
        else:
            self.connect_hardware()
        self.status.set(('SIMULATION — NO PHYSICAL MOVEMENT | synthetic calibration' if self.synthetic else
                         'SIMULATION — NO PHYSICAL MOVEMENT | saved calibration')
                        if simulate else f'REAL ROBOT — physical movement enabled after initialization | {port}')
        self.connection_banner.config(bg='#dbeafe' if simulate else '#fef3c7',fg='#1e40af' if simulate else '#92400e')
        self.log(f'miniArm XYZ Control v{APP_VERSION} — Developed by {DEVELOPER}; '+('SIMULATION, no physical movement' if simulate else f'REAL ROBOT, port {port}'))
        if self.synthetic:
            self.log('SIMULATION uses synthetic calibration; local calibration.json is ignored. Use --simulate --use-calibration to test your measured mapping.')
        elif self.cal:
            self.log(f'Saved calibration loaded: {self.cal.angle_convention} angle convention. Check angle references if IK limits seem wrong.')
        if not simulate:
            self.log('Connection leaves servos off. With calibration, arrange the unloaded arm upright before first enable; the upright command ignores sliders.')
        self.poll()  # process the emulator READY now; subsequent polls schedule themselves
        root.protocol('WM_DELETE_WINDOW',self.close)

    # =========================================================================
    # Window layout
    #   top:    yellow status box, then the mode buttons / STOP / Reload row
    #   left:   move-time row and feedback (APP, IK), the active mode's panel,
    #           then current estimate, live servo values and the upright row
    #   right:  preview camera, the 3D arm drawing (red), console (pink)
    # =========================================================================
    def _build(self):
        r=self.root
        try: r.geometry(WINDOW_SIZE); r.minsize(1150,760)
        except Exception: pass
        self.pending_servos=set(); self.knob_show_raw={j:False for j in range(5)}
        self._syncing=False; self._shown_panel=None

        top=tk.Frame(r,highlightthickness=2,highlightbackground=COLORS['status'],padx=8,pady=4)
        top.pack(fill='x',padx=10,pady=(8,4))
        self.mode_status=tk.StringVar(value='Not initialized')
        tk.Label(top,textvariable=self.mode_status,font=('Segoe UI',10,'bold'),
                 wraplength=1050,justify='center').pack()
        self.status=tk.StringVar()
        self.connection_banner=tk.Label(top,textvariable=self.status,font=('Segoe UI',10,'bold'),padx=12,pady=5)
        self.connection_banner.pack(fill='x',pady=3)
        self.version_info=tk.StringVar(value=f'v{APP_VERSION} | Developed by {DEVELOPER} | Firmware: waiting for connection')
        tk.Label(top,textvariable=self.version_info,fg='#475569').pack()
        connection_row=tk.Frame(top)
        connection_row.pack(pady=(3,0))
        tk.Label(connection_row,text='Arduino port:').pack(side='left')
        self.port_value=tk.StringVar(value=self.hardware_port)
        self.port_entry=tk.Entry(connection_row,textvariable=self.port_value,width=10)
        self.port_entry.pack(side='left',padx=5)
        self.reconnect_button=tk.Button(connection_row,text='Retry connection',command=self.connect_hardware)
        self.reconnect_button.pack(side='left',padx=4)
        if self.simulate: connection_row.pack_forget()

        bar=tk.Frame(r); bar.pack(fill='x',padx=10,pady=4)
        self.mode_buttons={}
        for mode in ('A','K','I'):
            button=tk.Button(bar,text=MODES[mode][0],width=15,pady=3,command=lambda m=mode:self.set_mode(m))
            button.pack(side='left',padx=3); self.mode_buttons[mode]=button
        self.stop_button=tk.Button(bar,text='STOP',bg='#c92a2a',fg='white',activebackground='#a61e1e',
                                   activeforeground='white',width=8,pady=3,font=('Segoe UI',9,'bold'),command=self.stop)
        self.stop_button.pack(side='left',padx=8)
        self.reload_button=tk.Button(bar,text='Reload calibration',width=17,pady=3,command=self.reload)
        self.reload_button.pack(side='left',padx=3)
        self.claw_settings_button=tk.Button(bar,text='Claw settings...',command=self.claw_settings,pady=3)
        self.claw_settings_button.pack(side='left',padx=3)
        self.init_button=tk.Button(bar,command=self.initialize,width=24,pady=2,
                                   highlightthickness=2,highlightbackground=COLORS['upright'])
        self.init_button.pack(side='left',padx=(14,0))
        self.init_text=tk.StringVar()
        tk.Label(bar,textvariable=self.init_text,fg='#7c4a1e',anchor='w',justify='left',
                 font=('Segoe UI',9,'bold')).pack(side='left',padx=8)

        body=tk.Frame(r); body.pack(fill='both',expand=True,padx=10,pady=(2,8))
        body.columnconfigure(0,weight=3,uniform='body'); body.columnconfigure(1,weight=2,uniform='body')
        body.rowconfigure(0,weight=1)
        left=tk.Frame(body); left.grid(row=0,column=0,sticky='nsew',padx=(0,10))
        right=tk.Frame(body); right.grid(row=0,column=1,sticky='nsew')
        self._build_left(left); self._build_right(right)
        self.update_init_text(); self.update_servo_line()
        self.draw(); self.refresh_controls()
        self.move_seconds.trace_add('write',lambda *args:self.target_edited())

    def _build_left(self,left):
        left.columnconfigure(0,weight=1); left.rowconfigure(2,weight=1)
        # Shared by the APP and IK panels: each panel shows its own copy of these rows
        self.move_seconds=tk.StringVar(value='2.0')
        self.action_hint=tk.StringVar()
        self.result_status=tk.StringVar(value='No IK preview calculated.')
        self.duration_inputs=[]; self.result_labels=[]
        # The active mode's panel
        holder=tk.Frame(left); holder.grid(row=2,column=0,sticky='nsew')
        holder.rowconfigure(0,weight=1); holder.columnconfigure(0,weight=1)
        self.panels={'A':self._build_app_panel(holder),'K':self._build_knob_panel(holder),'I':self._build_ik_panel(holder)}
        for panel in self.panels.values(): panel.grid(row=0,column=0,sticky='nsew'); panel.grid_remove()
        # Bottom: current estimate, live servo values (black line), upright row (brown)
        self.servo_line=tk.StringVar()
        tk.Label(left,textvariable=self.servo_line,font=('Consolas',10,'bold'),anchor='w').grid(row=5,column=0,sticky='ew')
        left.grid_rowconfigure(5,pad=6)

    def _panel_header(self,panel):
        """Move time + next-step + result rows, placed at the top of a mode panel."""
        frame=tk.Frame(panel)
        timing=tk.Frame(frame); timing.pack(anchor='w')
        tk.Label(timing,text='Move time (s, minimum):').pack(side='left')
        spin=tk.Spinbox(timing,from_=0.5,to=15,increment=.5,textvariable=self.move_seconds,width=5)
        spin.pack(side='left',padx=6); self.duration_inputs.append(spin)
        self.duration_input=spin
        tk.Label(timing,text='All joints move together; larger moves may take longer.').pack(side='left')
        tk.Label(frame,textvariable=self.action_hint,wraplength=760,justify='left',
                 fg='#475569',anchor='w').pack(fill='x',pady=(4,0))
        result=tk.Label(frame,textvariable=self.result_status,wraplength=760,
                        justify='left',anchor='w',font=('Segoe UI',9,'bold'))
        result.pack(fill='x',pady=(2,0)); self.result_labels.append(result)
        self.result_label=result
        return frame

    def _build_app_panel(self,holder):
        panel=tk.LabelFrame(holder,text='Manual servo commands — raw values, not DH joint angles',
                            fg=COLORS['app'],padx=10,pady=6)
        self._panel_header(panel).pack(fill='x',pady=(0,8))
        grid=tk.Frame(panel); grid.pack(anchor='w')
        self.sliders={}; self.servo_values={}; self.servo_entries={}; self.nudge_buttons=[]; self.send_buttons={}
        for col,j in enumerate(APP_ORDER):
            box=tk.Frame(grid,padx=12); box.grid(row=0,column=col,sticky='n')
            lo,hi=self.limit(j)
            s=tk.Scale(box,from_=hi,to=lo,orient='vertical',length=210,width=16,showvalue=0,
                       troughcolor='#d3f9d8',highlightthickness=0,command=lambda _v,j=j:self._slider_moved(j))
            s.set(90); s.grid(row=0,column=0,rowspan=5,sticky='ns'); self.sliders[j]=s
            tk.Label(box,text=JOINT_NAMES[j],fg=COLORS['app'],font=('Segoe UI',10,'bold')).grid(row=0,column=1,sticky='nw',padx=(6,0))
            up=tk.Button(box,text=f'+{NUDGE}',width=5,command=lambda j=j:self.nudge(j,NUDGE))
            up.grid(row=1,column=1,padx=(6,0),pady=(10,2))
            value=tk.StringVar(value='90'); self.servo_values[j]=value
            entry=tk.Entry(box,width=6,justify='center',textvariable=value,font=('Segoe UI',10,'bold'))
            entry.grid(row=2,column=1,padx=(6,0),pady=2)
            entry.bind('<Return>',lambda event,j=j:self.apply_typed(j))   # Enter applies, never sends
            entry.bind('<FocusOut>',lambda event,j=j:self.apply_typed(j))
            self.servo_entries[j]=entry
            down=tk.Button(box,text=f'-{NUDGE}',width=5,command=lambda j=j:self.nudge(j,-NUDGE))
            down.grid(row=3,column=1,padx=(6,0),pady=2)
            self.nudge_buttons+=[up,down]
            send=tk.Button(box,text=f'Send {JOINT_NAMES[j]}',width=12,
                           activebackground=COLORS['app'],activeforeground='white',
                           command=lambda j=j:self.send_servo(j))
            send.grid(row=5,column=0,columnspan=2,pady=(10,0)); self.send_buttons[j]=send
        self.manual_button=tk.Button(panel,text='Send manual sliders (all five)',width=40,pady=3,
                                     activebackground=COLORS['app'],activeforeground='white',
                                     command=self.manual)
        self.manual_button.pack(pady=(14,4))
        self.button_face=self.manual_button.cget('bg')      # the system's normal button colour
        tk.Label(panel,text='Green value box = changed but not sent yet. +/- and typing stop at the safe range.',
                 fg='#64748b').pack()
        return panel

    def _build_knob_panel(self,holder):
        panel=tk.LabelFrame(holder,text='Knob control — the knobs on the arm drive the servos',
                            fg=COLORS['knob'],padx=10,pady=6)
        grid=tk.Frame(panel); grid.pack(anchor='w',pady=6)
        self.knob_widgets={}
        for index,j in enumerate(KNOB_GRID):
            row,col=divmod(index,3)
            cell=tk.Frame(grid,padx=22,pady=8); cell.grid(row=row,column=col)
            canvas=tk.Canvas(cell,width=120,height=120,highlightthickness=0)
            canvas.pack()
            canvas.create_oval(10,10,110,110,outline=COLORS['knob'],width=5)
            needle=canvas.create_line(60,60,60,22,fill='#fab005',width=3,state='hidden')
            text=canvas.create_text(60,62,text='N/A' if j is None else '--',font=('Segoe UI',20,'bold'),fill='#1f2937')
            if j is None:
                button=tk.Button(cell,text='Nothing',width=14,state='disabled')
            else:
                button=tk.Button(cell,width=14,command=lambda j=j:self.toggle_knob_units(j))
            button.pack(pady=(4,0))
            self.knob_widgets[j]=(canvas,needle,text,button)
        self.knob_note=tk.StringVar(value='')
        tk.Label(panel,textvariable=self.knob_note,fg='#64748b').pack(anchor='w')
        self.update_knob_display()
        return panel

    def _build_ik_panel(self,holder):
        panel=tk.LabelFrame(holder,text='IK target — XYZ in millimetres',fg=COLORS['ik'],padx=10,pady=6)
        panel.columnconfigure(1,weight=1); panel.rowconfigure(1,weight=1)
        self._panel_header(panel).grid(row=0,column=0,columnspan=2,sticky='ew',pady=(0,8))
        controls=tk.Frame(panel); controls.grid(row=1,column=0,sticky='nw')
        self.entries=[]; self.target_values=[]
        for row,(name,value) in enumerate(zip(('X right (mm)','Y forward (mm)','Z up (mm)'),(-40,100,150))):
            tk.Label(controls,text=name,font=('Segoe UI',10,'bold')).grid(row=row,column=0,sticky='e',pady=6)
            variable=tk.StringVar(value=str(value)); self.target_values.append(variable)
            e=tk.Entry(controls,width=10,textvariable=variable,font=('Segoe UI',11),justify='center')
            e.grid(row=row,column=1,padx=(6,18),pady=6)
            variable.trace_add('write',lambda *args:self.target_edited())
            e.bind('<Return>',lambda event:self.calculate()); self.entries.append(e)
        blue=dict(activebackground=COLORS['ik'],activeforeground='white',width=18,pady=3)
        self.calculate_button=tk.Button(controls,text='Calculate / Preview',command=self.calculate,**blue)
        self.calculate_button.grid(row=0,column=2,pady=6)
        self.move_button=tk.Button(controls,text='Move Arm',state='disabled',command=self.move,**blue)
        self.move_button.grid(row=1,column=2,pady=6)
        self.claw_button=tk.Button(controls,text='Close claw',command=self.toggle_claw,**blue)
        self.claw_button.grid(row=2,column=2,pady=6)
        self.grip_buttons=[self.claw_button]
        self.save_button=tk.Button(controls,text='Save experiment log',command=self.save_log,
                                   activebackground=COLORS['ik'],activeforeground='white',pady=3)
        self.save_button.grid(row=3,column=0,columnspan=3,sticky='ew',pady=(16,0))
        task=tk.Frame(controls); task.grid(row=4,column=0,columnspan=3,sticky='ew',pady=(10,0))
        self.pp_settings_button=tk.Button(task,text='Cup settings...',command=self.pick_place_settings)
        self.pp_settings_button.pack(side='left',padx=2)
        self.pp_preview_button=tk.Button(task,text='Preview Pick & Place',command=self.preview_pick_place)
        self.pp_preview_button.pack(side='left',padx=2)
        self.pp_run_button=tk.Button(task,text='Run Pick & Place',command=self.run_pick_place,state='disabled')
        self.pp_run_button.pack(side='left',padx=2)
        self.pp_summary=tk.StringVar(value='Pickup = XYZ fields (TCP grasp point); cup [127, 0], rim Z=127 mm, opening 101.6 mm. Start with an empty claw.')
        tk.Label(controls,textvariable=self.pp_summary,wraplength=370,justify='left',fg='#0f766e').grid(
            row=5,column=0,columnspan=3,sticky='ew',pady=(5,0))
        self.error_canvas=tk.Canvas(panel,bg='white',width=360,height=280,highlightthickness=2,
                                    highlightbackground=COLORS['error'])
        self.error_canvas.grid(row=1,column=1,sticky='nsew',padx=(18,0))
        self.error_canvas.bind('<Configure>',lambda _:self.redraw_preview())
        return panel

    def _build_right(self,right):
        right.columnconfigure(0,weight=1); right.rowconfigure(1,weight=3); right.rowconfigure(3,weight=2)
        views=tk.Frame(right); views.grid(row=0,column=0,sticky='ew',pady=(4,2))
        tk.Label(views,text='Preview camera:').pack(side='left')
        self.preview_view=tk.StringVar(value='Rear right (Arduino side)')
        tk.OptionMenu(views,self.preview_view,*PREVIEW_CAMERAS,
                      command=lambda _:self.redraw_preview()).pack(side='left')
        self.canvas=tk.Canvas(right,width=480,height=360,bg='white',highlightthickness=2,
                              highlightbackground=COLORS['arm'])
        self.canvas.grid(row=1,column=0,sticky='nsew')
        self.canvas.bind('<Configure>',lambda _:self.redraw_preview())
        self.readout=tk.StringVar(value='Current estimate: not initialized')
        tk.Label(right,textvariable=self.readout,font=('Consolas',9),anchor='w',justify='left'
                 ).grid(row=2,column=0,sticky='ew',pady=(4,0))
        console=tk.Frame(right,highlightthickness=2,highlightbackground=COLORS['console'])
        console.grid(row=3,column=0,sticky='nsew',pady=(6,0))
        self.terminal=scrolledtext.ScrolledText(console,width=60,height=10,state='disabled',
                                                font=('Consolas',9),relief='flat')
        self.terminal.pack(fill='both',expand=True)

    def show_panel(self,mode):
        """Swap the left panel; the move-time and feedback rows hide in KNOB mode."""
        panels=getattr(self,'panels',None)
        if not panels or self._shown_panel==mode: return
        for key,panel in panels.items():
            if key==mode: panel.grid()
            else: panel.grid_remove()
        self._shown_panel=mode
        if mode=='K': self.update_knob_display()

    def refresh_controls(self):
        """One place for mode highlighting and initialization/motion gating."""
        waiting=getattr(self,'stop_pending',False)
        idle=self.ready and not self.busy and not waiting
        enabled=idle and self.initialized
        def state(widget,allow):
            if widget is not None and hasattr(widget,'config'): widget.config(state='normal' if allow else 'disabled')
        can_connect=not self.simulate and not self.busy and not waiting and not self._connecting and not self.initialized
        state(getattr(self,'reconnect_button',None),can_connect)
        state(getattr(self,'port_entry',None),can_connect)
        for mode,button in getattr(self,'mode_buttons',{}).items():
            selected=self.initialized and mode==self.mode
            color=MODES[mode][1] if selected else '#e5e7eb'
            button.config(bg=color,activebackground=color,fg='white' if selected else '#334155',
                          relief='sunken' if selected else 'raised',
                          font=('Segoe UI',9,'bold' if selected else 'normal'))
            state(button,enabled)
        state(getattr(self,'init_button',None),idle and not (self.initialized and self.mode=='K'))
        if hasattr(self,'init_button'): self.init_button.config(text=self.upright_button_text())
        state(getattr(self,'manual_button',None),enabled and self.mode=='A')
        state(getattr(self,'calculate_button',None),enabled and self.mode=='I' and self.cal is not None)
        state(getattr(self,'move_button',None),enabled and self.mode=='I' and self.preview is not None and self.supports_timed_motion())
        pp_allowed=enabled and self.mode=='I' and self.cal is not None and self.supports_timed_motion()
        state(getattr(self,'pp_settings_button',None),enabled and self.mode=='I')
        state(getattr(self,'pp_preview_button',None),pp_allowed)
        state(getattr(self,'pp_run_button',None),pp_allowed and self._pp_preview is not None)
        for button in getattr(self,'grip_buttons',[]):state(button,enabled and self.mode!='K' and self.cal is not None)
        state(getattr(self,'save_button',None),enabled)
        for spin in getattr(self,'duration_inputs',[getattr(self,'duration_input',None)]):state(spin,enabled and self.mode!='K')
        state(getattr(self,'reload_button',None),not self.busy and not waiting)
        claw_edit=not self.busy and not waiting and self.cal is not None
        state(getattr(self,'claw_settings_button',None),claw_edit)
        if self._claw_controls is not None:
            for widget in self._claw_controls['edit']: state(widget,claw_edit)
            for widget in self._claw_controls['test']:
                state(widget,claw_edit and enabled and self.mode!='K' and self.supports_timed_motion())
            state(self._claw_controls['save'],claw_edit and not self.synthetic)
            self._claw_controls['current'].set(
                f'Current S1 command estimate: {self.positions[CLAW]:.1f}' if self.initialized else 'Arm not initialized; tests are disabled.')
        for slider in getattr(self,'sliders',{}).values():state(slider,enabled and self.mode=='A')
        app_ok=enabled and self.mode=='A'
        for w in [*getattr(self,'servo_entries',{}).values(),*getattr(self,'send_buttons',{}).values(),
                  *getattr(self,'nudge_buttons',[])]:state(w,app_ok)
        if hasattr(self,'claw_button') and self.cal is not None:
            self.claw_button.config(text='Open claw' if self.claw_is_closed() else 'Close claw')
        if hasattr(self,'panels'): self.show_panel(self.mode if self.initialized else 'A')
        for entry in getattr(self,'entries',[]):state(entry,enabled and self.mode=='I')
        if hasattr(self,'mode_status'):
            if waiting: text='STOP requested — awaiting Arduino confirmation'
            elif not self.ready: text=self.connection_state
            elif not self.initialized:
                text='Initializing — waiting for controller acknowledgement' if self.busy else f'Not initialized — click {self.init_button_name()} to enable controls'
            else:
                suffix=' — STOPPED (servos holding)' if getattr(self,'stopped',False) else (' — moving' if self.busy else '')
                text='Active mode: '+MODES[self.mode][0]+suffix
            self.mode_status.set(text)
        if hasattr(self,'action_hint'):
            if waiting: hint='Waiting for STOP confirmation and the command-position report.'
            elif not self.ready: hint=self.connection_state
            elif not self.initialized:
                hint=('Initialization in progress; STOP cancels remaining servo enables.' if self.busy else
                      f'Servos are still on. Click {self.init_button_name()} to return upright smoothly and re-enable the controls.'
                      if getattr(self,'servos_on',False) else
                      f'Support the unloaded arm, then click {self.init_button_name()}. Servos enable one at a time; each may still move quickly.')
            elif self._pp_planning: hint='Planning all pick-and-place stages; no servo commands sent. STOP cancels planning.'
            elif self._pp_running: hint='Pick & Place: '+self._pp_current_label+'. STOP cancels the rest and keeps the current claw command.'
            elif self.busy: hint=f'Moving: {len(getattr(self,"plan",[]))} waypoint(s) queued after the active command. STOP cancels the remaining plan.'
            elif self.mode=='K': hint='Knobs drive the arm. Choose APP or IK before requesting an upright move.'
            elif self.mode=='A': hint='Move a slider, use +/-, or type a value (Enter does not send). Then Send that servo, or Send all five. Choose IK CONTROL to enter an XYZ target.'
            elif self.preview is not None and not self.supports_timed_motion(): hint='Upload the updated v2.0.0 Arduino sketch to enable coordinated movement.'
            elif self.preview is not None: hint='Preview is ready. Move Arm sends one final target; Arduino generates the smooth motion.'
            else: hint='Enter XYZ in mm, then Calculate / Preview (or press Enter). This does not move the arm.'
            self.action_hint.set(hint)

    def connect_hardware(self):
        """Start/retry the handshake without blocking Tk or replaying motion."""
        if self.simulate or self._connecting or self.busy or self.initialized:
            return
        if self.link is not None:
            self.link.close(); self.link=None
        # A new queue isolates late messages from the previous serial worker.
        self.inbox=queue.Queue()
        self.ready=False; self.initialized=False; self.pending=None; self.plan=[]
        self.servos_on=False; self._recovering=False
        self.stop_pending=False; self.stop_acknowledged=False
        self.pending_firmware=None; self.firmware_version='unknown'; self.timed_supported=False
        self.sequential_supported=False
        self.knobs=None; self.knob_baseline=None
        self.invalidate(clear_display=True)
        self.hardware_port=self.port_value.get().strip()
        if serial is None:
            self.connection_state='pyserial missing: run python -m pip install pyserial, then restart.'
        elif not self.hardware_port:
            self.connection_state='Enter the Arduino COM port, then Retry connection.'
        else:
            self.connection_state=f'Connecting to {self.hardware_port}; waiting for Arduino reset/boot...'
            self._connecting=True
            self.link=SerialLink(self.hardware_port,BAUD_RATE,self.inbox)
        self.status.set(f'REAL ROBOT - initialization required | {self.hardware_port}')
        self.version_info.set(f'v{APP_VERSION} | Developed by {DEVELOPER} | Firmware: waiting for handshake')
        self.log(self.connection_state)
        self.refresh_controls()

    def init_button_name(self):
        return '"'+self.upright_button_text()+'"'

    def upright_button_text(self):
        if not self.cal: return 'Enable raw neutral (uncalibrated)'
        return 'Slowly straighten upright' if (self.initialized or getattr(self,'servos_on',False)) else 'Enable servos one by one'

    def supports_timed_motion(self):
        return getattr(self,'timed_supported',False) or getattr(getattr(self,'link',None),'supports_timed',False)

    def chosen_duration(self,start,end):
        try:value=float(self.move_seconds.get()) if hasattr(self,'move_seconds') else 2.
        except ValueError:raise ValueError('Move time must be a number between 0.5 and 15 seconds') from None
        return ik.motion_duration_ms(start,end,value)

    def upright_duration(self,start,end):
        """Cubic peak speed is 1.5 * largest raw-command change / duration."""
        change=max(abs(float(a)-float(b)) for a,b in zip(start,end))
        duration=max(self.chosen_duration(start,end),
                     math.ceil(UPRIGHT_MIN_SECONDS*1000),
                     math.ceil(1500*change/UPRIGHT_MAX_SERVO_SPEED))
        if duration>25000: raise ValueError('Upright move needs more than the firmware duration limit')
        return duration

    def feedback(self,message,kind='info'):
        self.feedback_message=message;self.feedback_kind=kind
        if hasattr(self,'result_status'): self.result_status.set(message)
        for label in getattr(self,'result_labels',[getattr(self,'result_label',None)]):
            if label is not None: label.config(fg={'info':'#475569','success':'#15803d','error':'#b91c1c'}[kind])

    def log(self,text):
        self.terminal.config(state='normal'); self.terminal.insert('end',text+'\n')
        self.terminal.see('end'); self.terminal.config(state='disabled')

    def limit(self,j):
        return self.cal.limits(j) if self.cal else (0,180)

    def start_pose(self):
        """Fixed upright reference, independent of model seed and manual sliders.

        Without calibration only raw neutral is known, not a physical posture.
        Never clip an unavailable upright pose into a different configuration.
        """
        if self.cal:
            q=ik.model_angles(np.deg2rad([90.,90.,0.,0.]))   # real upright, claw in line with the arm
            if np.any(q < self.m.limits[:,0]) or np.any(q > self.m.limits[:,1]):
                raise ValueError('upright angles are outside calibrated limits')
            pose=ik.q_to_servo(self.cal,q,self.cal.claw_open)
            if ik.clearance(self.m,ik.servo_to_q(self.cal,pose))[0] <= 0:
                raise ValueError('upright pose violates configured clearance')
            return pose
        return [90]*5

    def send(self,line):
        if not self.ready or not self.link: raise ValueError('Arduino is not ready')
        word=line.split()[0] if line.split() else ''
        if word in ('T','M','J','INIT','SINIT'): self._last_command=word
        if word=='T':   # remember where a smooth move started (for the finish)
            self._finish_from=list(self.positions)
        self.link.send(line)

    # -------------------------------------------------------------------------
    # Overshoot-and-return finish (see FINISH_OVERSHOOT at the top)
    # -------------------------------------------------------------------------
    def finish_poses(self,start,target):
        """[side pose, target] for the FINISH_JOINTS, or [] if nothing to do.

        First choice: step PAST the target, then return. If that pose is not
        clear (e.g. it would push the claw toward the table), back OFF the
        same amount instead and return, which still ends with a firm step in
        the original direction of travel.
        """
        if FINISH_OVERSHOOT<=0 or not self.cal: return []
        names={'base':BASE,'shoulder':ARM,'elbow':FOREARM,'wrist':WRIST}
        moved=[(name,names[name],float(target[names[name]])-float(start[names[name]]))
               for name in FINISH_JOINTS
               if abs(float(target[names[name]])-float(start[names[name]]))>=FINISH_MIN_MOVE]
        if not moved: return []
        reasons=[]
        for side,label in ((1,'past'),(-1,'back-off')):
            pose=list(target); changed=[]
            for name,j,delta in moved:
                lo,hi=self.limit(j)
                value=min(max(float(target[j])+side*math.copysign(FINISH_OVERSHOOT,delta),lo),hi)
                if abs(value-float(target[j]))>=1: pose[j]=value; changed.append(name)
            if not changed: continue
            try:   # both short legs must clear the table, base and arm
                ik.validate_servo_leg(self.m,self.cal,list(target),pose)
                ik.validate_servo_leg(self.m,self.cal,pose,list(target))
            except ValueError as e:
                reasons.append(f'{label}: {e}'); continue
            if side<0: self.log(f'Finish: overshoot not clear here; backing off and returning ({", ".join(changed)}).')
            return [pose,list(target)]
        if reasons: self.log('Finish skipped: '+'; '.join(reasons))
        return []

    def continue_finish(self,completed):
        """Called on each OK. Returns True while a finish step is in flight."""
        steps=getattr(self,'_finish_steps',None)
        if steps:
            self.pending=steps.pop(0); self.started=time.monotonic()
            self.send('M '+' '.join(f'{float(v):.6g}' for v in self.pending))
            return True
        start=getattr(self,'_finish_from',None); self._finish_from=None
        if completed!='T' or start is None or self.stopped: return False
        poses=self.finish_poses(start,self.positions)
        if not poses: return False
        self._finish_steps=poses[1:]
        self.pending=poses[0]; self.started=time.monotonic()
        self.send('M '+' '.join(f'{float(v):.6g}' for v in self.pending))
        return True

    def invalidate(self,clear_display=False):
        """Invalidate motion without erasing the last calculated drawing."""
        self.preview=None
        self._pp_preview=None
        self.move_button.config(state='disabled')
        if clear_display:
            self.cancel_pick_place()
            self._pp_plot=None
            self._preview_plot=None
            self._target_points=None
            self._app_target_pose=None
            self._app_target_points=None
        retained=getattr(self,'_preview_plot',None) is not None
        message='Last calculated preview retained — calculate again before Move Arm.' if retained else 'Calculate / Preview prepares a move; editing inputs does not move the arm.'
        self.feedback(message if self.mode=='I' else '')
        if hasattr(self,'canvas'):self.redraw_preview()
        self.refresh_controls()

    def set_mode(self,mode):
        if not self.ready or not self.initialized or getattr(self,'stop_pending',False):
            self.log('Initialize the arm before selecting a mode.'); return
        if self.busy:
            self.log('Press STOP before changing mode.'); return
        self.invalidate(); self.mode=mode; self.stopped=False
        self.reset_pending()
        if mode!='I': self.feedback('')
        self.knob_baseline=list(self.knobs) if self.knobs else None
        if self.ready:
            try: self.send('MODE '+mode)
            except Exception as e: self.log(str(e))
        self.log('Mode: '+MODES[mode][0]); self.refresh_controls()

    def initialize(self):
        if not self.ready or self.busy or getattr(self,'stop_pending',False): return
        servos_on=getattr(self,'servos_on',False)
        if self.initialized and self.mode=='K':
            message='Choose APP or IK before an upright move; knob mode would move away from upright.'
            self.log(message); self.feedback(message); return
        try:
            pose=self.start_pose()  # fixed reference; never read manual sliders
            if self.cal:
                self.log('Upright calibration conversion: '+', '.join(
                    f'{JOINT_NAMES[mc.SERVO_INDEX[name]]} q={q:g} deg -> S{mc.SERVO_INDEX[name]+1} raw {pose[mc.SERVO_INDEX[name]]}'
                    for name,q in zip(mc.JOINTS,[90,90,0,0])))
            if self.initialized or servos_on:
                # Servos already on (normal case, or after a calibration reload / ERR /
                # timeout): the Arduino refuses SINIT, so return upright with a smooth
                # timed move. Success re-initializes the GUI.
                recovering=not self.initialized
                pose[CLAW]=self.positions[CLAW]  # do not release a held object
                if not self.supports_timed_motion():
                    raise ValueError('Gentle upright motion requires the v2.0.0 timed-motion firmware')
                if self.cal:
                    ik.validate_coordinated_leg(self.m,self.cal,self.positions,pose)
                duration=self.upright_duration(self.positions,pose)
                self.invalidate()
                if recovering:   # limits may have changed with a reloaded calibration
                    limits=' '.join(f'{lo} {hi}' for lo,hi in (self.limit(j) for j in range(5)))
                    self.send(f'CFG {limits} {STEP_MS} {SETTLE_MS}')
                    if self.mode=='K':   # knobs would fight the upright move; MODE A also freezes the arm
                        self.send('MODE A'); self.mode='A'
                    self._recovering=True
                    self.log('Servos are already on: straightening upright smoothly to re-initialize.')
                self.log(f'Gentle upright return: {duration/1000:.2f} s; keeping '+MODES[self.mode][0]+'.')
                if self.mode=='A':
                    self.capture_app_target(pose); self.redraw_preview()
                self.launch([pose],command='T',duration_ms=duration)
                return
            simulated=getattr(getattr(self,'link',None),'simulated',False)
            if not simulated and not self.sequential_supported:
                raise ValueError('Upload the included firmware v2.1.0 (SEQINIT) for one-servo-at-a-time enable; old INIT would switch on all servos together')
            self.invalidate()
            limits=' '.join(f'{lo} {hi}' for lo,hi in (self.limit(j) for j in range(5)))
            self.send(f'CFG {limits} {STEP_MS} {SETTLE_MS}')
            self.mode='A'; self.send('MODE A')
            if simulated:
                self.log('Initializing emulator at upright reference; no physical movement.')
                self.launch([pose],command='INIT')
            else:
                self.log('Sequential first enable: shoulder -> elbow -> wrist -> base -> claw. '
                         f'{STARTUP_SERVO_PAUSE_MS/1000:g} s pause after each. Support the unloaded arm; '
                         'each first enable can still move quickly. Claw uses its calibrated startup opening.')
                self.feedback('Enabling one servo at a time; support the unloaded arm. STOP cancels remaining enables.')
                self.launch([pose],command='SINIT')
        except Exception as e:
            message='Cannot straighten/enable arm: '+str(e)
            self.log(message); self.feedback(message,'error')

    def launch(self,poses,command='J',duration_ms=None):
        if command=='SINIT' and not self.sequential_supported:
            raise ValueError('Sequential first enable requires firmware 2.1.0 SEQINIT')
        if command=='T':
            if not self.supports_timed_motion():
                raise ValueError('Upload the updated v2.0.0 Arduino sketch for smooth timed movement')
            if duration_ms is None or not np.isfinite(duration_ms) or duration_ms!=int(duration_ms) or not 500<=duration_ms<=25000:
                raise ValueError('Timed motion requires an integer duration of 500..25000 ms')
        self.active_duration_ms=duration_ms
        self.stopped=False
        self.knob_baseline=list(self.knobs) if getattr(self,'knobs',None) else None
        self.plan=[list(p) for p in poses]; self.active=command
        self.busy=True; self.refresh_controls(); self.next_waypoint()

    def next_waypoint(self):
        if not self.plan:
            self.busy=False; self.pending=None
            self.log('Command sequence finished (estimated position; no encoder feedback).'); self.refresh_controls(); return
        self.pending=self.plan.pop(0); self.started=time.monotonic()
        prefix=(f'T {self.active_duration_ms}' if self.active=='T' else
                f'SINIT {STARTUP_SERVO_PAUSE_MS}' if self.active=='SINIT' else self.active)
        self.send(prefix+' '+' '.join(f'{float(v):.6g}' for v in self.pending))

    def poll(self):
        if getattr(self,'_closed',False): return
        previous=getattr(self,'_poll_id',None)
        if previous is not None:
            try: self.root.after_cancel(previous)
            except tk.TclError: pass
            self._poll_id=None
        if self.link: self.link.tick()
        self.poll_pick_place()
        while not self.inbox.empty():
            line=self.inbox.get(); parts=line.split()
            if not parts: continue
            if parts[0]=='LINK':
                self.connection_state=line[5:]
                self.log(self.connection_state)
            elif parts[0]=='LINK_ERROR':
                self.ready=False; self.initialized=False; self._connecting=False
                self.servos_on=False; self._recovering=False
                self.busy=False; self.pending=None; self.plan=[]
                self.stop_pending=False; self.stop_acknowledged=False
                self.connection_state=line[len('LINK_ERROR '):]
                self.timed_supported=False; self.pending_firmware=None
                self.sequential_supported=False
                self.invalidate(clear_display=True)
                self.log(self.connection_state)
                self.feedback(self.connection_state,'error')
                self.version_info.set(f'v{APP_VERSION} | Developed by {DEVELOPER} | Firmware: disconnected')
            elif parts[0]=='FW' and len(parts)>=2:
                self.firmware_version=parts[1];self.timed_supported='TIMED' in parts[2:]
                self.sequential_supported='SEQINIT' in parts[2:]
                self.pending_firmware=(self.firmware_version,self.timed_supported,self.sequential_supported)
                if hasattr(self,'version_info'):
                    self.version_info.set(f'v{APP_VERSION} | Developed by {DEVELOPER} | Firmware {self.firmware_version}')
            elif line.endswith('READY'):
                self._connecting=False
                self.connection_state='Connected; initialize the arm explicitly.'
                self.ready=True; self.initialized=False; self.busy=False
                self.servos_on=False; self._recovering=False     # an Arduino reset switches servos off
                if not getattr(self.link,'simulated',False):
                    self.firmware_version,self.timed_supported,self.sequential_supported=getattr(self,'pending_firmware',None) or ('unknown',False,False)
                    self.pending_firmware=None
                    if hasattr(self,'version_info'):
                        suffix=self.firmware_version if self.timed_supported else 'unknown/older — upload v2.0.0 for smooth IK'
                        self.version_info.set(f'v{APP_VERSION} | Developed by {DEVELOPER} | Firmware: {suffix}')
                self.stopped=False; self.stop_pending=False; self.stop_acknowledged=False; self.knob_takeover_pending=False
                self.plan=[]; self.pending=None; self.invalidate(clear_display=True)
                if getattr(self.link,'simulated',False):
                    self.firmware_version=self.link.firmware_version
                    if hasattr(self,'version_info'):self.version_info.set(f'v{APP_VERSION} | Developed by {DEVELOPER} | Emulator {self.firmware_version}')
                    self.log('Emulator READY. Preparing upright reference for direct IK preview.')
                    self.initialize()
                else:
                    self.log('Arduino READY. Servos off; initialize explicitly.')
            elif parts[0]=='INITSEQ':
                self.log('Startup: '+line)
                self.feedback('Sequential startup: '+line+'; STOP cancels remaining enables.')
            elif parts[0]=='P' and len(parts)==6:
                try:
                    reported=[float(v) for v in parts[1:]]
                    if not all(math.isfinite(v) and 0<=v<=180 for v in reported): raise ValueError
                except ValueError:
                    self.log('Ignored malformed serial position report.'); continue
                self.positions=reported; self.update_position()
                if getattr(self,'stop_pending',False) and self.stop_acknowledged:
                    self.stop_pending=False
                    if self.initialized:
                        message='STOP confirmed - commands frozen, servos holding. Calculate again before an IK move.'
                    else:
                        message='STOP confirmed - remaining enables cancelled; enabled joints hold, others remain off. Support the arm.'
                    self.log(message); self.feedback(message)
                    if getattr(self,'knob_takeover_pending',False):
                        self.knob_takeover_pending=False
                        self.set_mode('K')
            elif line=='STOPPED':
                self.stop_acknowledged=True
            elif parts[0]=='MODE' and len(parts)==2 and parts[1] in MODES:
                self.mode=parts[1]
            elif line=='OK' and self.pending is not None:
                self.positions=list(self.pending); self.commanded=list(self.pending)
                if self.active in ('INIT','SINIT'): self.initialized=True; self.servos_on=True
                elif self.active=='T' and getattr(self,'_recovering',False):
                    self.initialized=True; self._recovering=False   # straightened upright again
                completed=getattr(self,'_last_command','')
                self.pending=None; self.update_position()
                try:
                    if self.continue_finish(completed): continue
                    if self.active=='PICKPLACE': self.pick_place_step_done()
                    else: self.next_waypoint()
                    if self.active=='INIT' and not self.busy and getattr(self.link,'simulated',False):
                        self.set_mode('I')
                        self.log('Simulation ready for XYZ Calculate / Preview; no upright click required.')
                except Exception as e: self.fail(str(e))
            elif parts[0]=='K' and len(parts)==6:
                try:
                    knobs=[int(v) for v in parts[1:]]
                    if not all(0<=v<=1023 for v in knobs): raise ValueError
                except ValueError:
                    self.log('Ignored malformed serial knob report.'); continue
                self.knobs=knobs
                if getattr(self,'_shown_panel',None)=='K': self.update_knob_display()
                if self.knob_baseline is None: self.knob_baseline=list(self.knobs)
                threshold=KNOB_TAKEOVER_DEG/180*1023
                if self.initialized and not getattr(self,'stopped',False) and self.mode!='K' and any(abs(a-b)>threshold for a,b in zip(self.knobs,self.knob_baseline)):
                    self.stop(); self.knob_takeover_pending=True
                    self.log('Knob takeover: cancelling IK; knob mode starts after STOP confirmation.')
            elif parts[0]=='ERR': self.fail(line)
        if self.busy and not self._pp_planning and time.monotonic()-self.started>30:
            self.stop(); self.log('Move timed out. Check connection and pose before reinitializing.')
            self.initialized=False
        self.refresh_controls()
        self._poll_id=self.root.after(20,self.poll)

    def fail(self,message):
        self.cancel_pick_place()
        self.log(message); self.plan=[]; self.pending=None; self.busy=False; self.invalidate()
        self.initialized=False; self.stop_pending=False; self._recovering=False
        self._finish_steps=None; self._finish_from=None
        if self.ready and self.link:
            try: self.link.send('STOP')
            except Exception: self.ready=False
        self.refresh_controls()

    def stop(self):
        # Cancel future sends first; send STOP before redrawing anything.
        self.cancel_pick_place()
        self.plan=[]; self.pending=None; self.busy=False; self.stopped=True
        self._finish_steps=None; self._finish_from=None
        self.stop_pending=False; self.stop_acknowledged=False; self.knob_takeover_pending=False
        if self.ready:
            try:
                if self.mode=='K': self.link.send('MODE A'); self.mode='A'
                self.stop_pending=True
                self.link.send('STOP')
            except Exception as e: self.fail(str(e))
        self.invalidate()
        self.log('STOP requested; queued waypoints cancelled. Awaiting confirmation; this does not remove servo power.')
        self.feedback('STOP requested — waiting for the controller; servo power remains on.')
        self.refresh_controls()

    # -------------------------------------------------------------------------
    # Editing XYZ or move time disables Move Arm, retaining the last drawing.
    # Automatic recalculation is off by default; Calculate / Preview runs IK.
    # -------------------------------------------------------------------------
    def target_edited(self):
        self.invalidate()
        pending=getattr(self,'_auto_job',None)
        if pending is not None:
            try: self.root.after_cancel(pending)
            except Exception: pass
            self._auto_job=None
        if AUTO_PREVIEW_MS>0 and self.mode=='I':
            self._auto_job=self.root.after(AUTO_PREVIEW_MS,self.auto_preview)

    def auto_preview(self):
        self._auto_job=None
        if (self.mode!='I' or not self.initialized or not self.cal or self.busy
                or getattr(self,'stop_pending',False) or self.preview is not None): return
        try:   # quietly wait while a box is mid-edit ("", "-", "1.")
            values=[float(e.get()) for e in self.entries]
            if not np.isfinite(values).all(): return
        except ValueError: return
        self.calculate(auto=True)

    def apply_claw_settings(self,opened,closed,save=False):
        """Changing presets sends nothing and invalidates old executable plans."""
        if self.busy or self.stop_pending:
            raise ValueError('Wait for motion/STOP confirmation before changing claw settings')
        if save and self.synthetic:
            raise ValueError('Synthetic simulation settings are session-only; hardware calibration was not changed')
        updated=(save_claw_calibration(self.cal,opened,closed,calibration_path()) if save else
                 claw_calibration(self.cal,opened,closed))
        self.cal=updated
        self._pp_plot=None; self.invalidate()
        self.update_init_text(); self.refresh_controls()
        destination='saved to calibration.json (backup: calibration.json.before_claw_edit)' if save else 'applied for this session only'
        message=f'Claw S1 Open={updated.claw_open}, Closed={updated.claw_closed}; {destination}. No motion sent.'
        self.log(message); self.feedback(message)
        return updated

    def claw_settings(self):
        if self.busy or self.stop_pending or self.cal is None: return
        if self._claw_dialog is not None and self._claw_dialog.winfo_exists():
            self._claw_dialog.lift(); return
        dialog=tk.Toplevel(self.root); self._claw_dialog=dialog
        dialog.title('Claw Open / Closed presets'); dialog.transient(self.root)
        low,high=self.cal.limits(CLAW)
        tk.Label(dialog,text=f'S1 raw servo values; calibrated safe range {low}..{high}',
                 font=('Segoe UI',10,'bold')).grid(row=0,column=0,columnspan=3,padx=12,pady=10)
        opened=tk.StringVar(value=str(self.cal.claw_open)); closed=tk.StringVar(value=str(self.cal.claw_closed))
        entries=[]
        for row,(label,var) in enumerate((('Open',opened),('Closed',closed)),1):
            tk.Label(dialog,text=label).grid(row=row,column=0,sticky='w',padx=12,pady=5)
            entry=tk.Spinbox(dialog,from_=low,to=high,increment=1,textvariable=var,width=9)
            entry.grid(row=row,column=1,padx=8,pady=5); entries.append(entry)
        message=tk.StringVar(value='Edit values, then Apply. Typing alone does not move the claw.')
        def apply(save=False,test=None):
            try:
                if test is not None and (not self.ready or not self.initialized or self.mode=='K' or not self.supports_timed_motion()):
                    raise ValueError('Initialize and select APP or IK with timed firmware before testing the claw')
                self.apply_claw_settings(opened.get(),closed.get(),save)
                if test is not None:
                    self.grip(test)
                    message.set((('Testing Closed' if test else 'Testing Open')+'; presets applied for this session. Save to keep them.')
                                if self.busy else 'Test could not start; check the controller log.')
                else: message.set('Saved to calibration.json.' if save else 'Applied for this session; Save to keep these presets.')
            except (ValueError,mc.CalibrationError,OSError) as error: message.set(str(error))
        test_open=tk.Button(dialog,text='Test Open',command=lambda:apply(test=False))
        test_open.grid(row=1,column=2,padx=12,pady=5)
        test_closed=tk.Button(dialog,text='Test Closed',command=lambda:apply(test=True))
        test_closed.grid(row=2,column=2,padx=12,pady=5)
        current=tk.StringVar()
        tk.Label(dialog,textvariable=current,fg='#475569').grid(row=3,column=0,columnspan=3,pady=5)
        apply_button=tk.Button(dialog,text='Apply (session only)',command=apply)
        apply_button.grid(row=4,column=0,columnspan=2,padx=12,pady=8)
        save_button=tk.Button(dialog,text='Save to calibration',command=lambda:apply(save=True))
        save_button.grid(row=4,column=2,padx=12,pady=8)
        note=('Values are raw servo commands, not DH angles or gripping force. Use small adjustments. '
              'New values apply to Open/Close, startup and Pick & Place. Safe limits are changed only in the calibration tool.')
        if self.synthetic: note+=' Simulation edits are session-only; your hardware file is not changed.'
        tk.Label(dialog,text=note,wraplength=470,justify='left',fg='#475569').grid(row=5,column=0,columnspan=3,padx=12,pady=8)
        tk.Label(dialog,textvariable=message,wraplength=470,justify='left',fg='#0f766e').grid(row=6,column=0,columnspan=3,padx=12,pady=(0,12))
        self._claw_controls={'edit':entries+[apply_button],'test':[test_open,test_closed],
                             'save':save_button,'current':current,'values':(opened,closed),'message':message}
        def close_dialog():
            self._claw_controls=None; self._claw_dialog=None; dialog.destroy()
        dialog.protocol('WM_DELETE_WINDOW',close_dialog)
        self.refresh_controls()

    def pick_place_settings(self):
        if self.busy: return
        dialog=tk.Toplevel(self.root); dialog.title('Cup and block settings (mm)')
        dialog.transient(self.root)
        fields=[('x','Cup center X',self._pp_settings.x),('y','Cup center Y',self._pp_settings.y),
                ('rim_z','Cup rim Z (base is Z=0)',self._pp_settings.rim_z),
                ('diameter','Opening diameter',self._pp_settings.diameter),
                ('lift','Lift / transport clearance',self._pp_settings.lift),
                ('object_radius','Held-block radius about TCP',self._pp_settings.object_radius),
                ('release_margin','Extra release margin above block',self._pp_settings.release_margin),
                ('dwell','Settling / grip pause (seconds)',self._pp_settings.dwell)]
        values={}
        for row,(key,label,value) in enumerate(fields):
            tk.Label(dialog,text=label).grid(row=row,column=0,sticky='w',padx=10,pady=4)
            variable=tk.StringVar(value=f'{value:g}'); values[key]=variable
            tk.Entry(dialog,textvariable=variable,width=12).grid(row=row,column=1,padx=10,pady=4)
        note=('5 inches = 127 mm; 4-inch opening = 101.6 mm. XYZ uses the base-axis/table origin. '
              'Block radius must enclose the held object around the TCP; measure it. '
              'XYZ IK does not constrain claw pitch or verify grasping. Start with an empty claw.')
        tk.Label(dialog,text=note,wraplength=430,justify='left').grid(row=8,column=0,columnspan=2,padx=10,pady=8)
        message=tk.StringVar()
        tk.Label(dialog,textvariable=message,fg='#b91c1c',wraplength=430).grid(row=9,column=0,columnspan=2)
        def save():
            if self.busy:
                message.set('Wait for the task to finish or press STOP before changing settings.'); return
            try: settings=pp.CupSettings(**{k:float(v.get()) for k,v in values.items()}).validate()
            except ValueError as error: message.set(str(error)); return
            self._pp_settings=settings; self._pp_plot=None; self.invalidate()
            self.pp_summary.set(f'Pickup = XYZ fields (TCP grasp point); cup [{settings.x:g}, {settings.y:g}], rim Z={settings.rim_z:g} mm; opening {settings.diameter:g} mm. Start with an empty claw.')
            dialog.destroy()
        tk.Button(dialog,text='Apply settings',command=save).grid(row=10,column=0,columnspan=2,pady=10)

    def cancel_pick_place(self):
        self._pp_cancel.set(); self._pp_generation+=1
        if self._pp_delay is not None:
            try: self.root.after_cancel(self._pp_delay)
            except tk.TclError: pass
        self._pp_delay=None; self._pp_steps=[]
        self._pp_running=False; self._pp_planning=False; self._pp_preview=None

    def preview_pick_place(self):
        if self.busy or self.stop_pending or self.mode!='I' or not self.initialized or not self.cal:
            return
        if not self.supports_timed_motion():
            self.feedback('Timed-motion firmware is required for Pick & Place.','error'); return
        try:
            pick=np.array([float(e.get()) for e in self.entries])
            seconds=float(self.move_seconds.get())
            self._pp_settings.validate()
            ik.motion_duration_ms(self.positions,self.positions,seconds)
        except ValueError as error: self.feedback(str(error),'error'); return
        self.cancel_pick_place(); self.invalidate()
        self._pp_cancel=threading.Event(); cancel=self._pp_cancel
        generation=self._pp_generation; start=list(self.positions); settings=self._pp_settings
        self._pp_planning=True; self.busy=True; self.started=time.monotonic()
        self.feedback('Planning the full task; nothing moves until Run Pick & Place.')
        self.refresh_controls()
        def worker():
            try:
                plan=pp.build_plan(self.m,self.cal,start,pick,settings,seconds,cancel.is_set,
                    lambda label:self._pp_messages.put((generation,'progress',label)))
                self._pp_messages.put((generation,'ready',plan))
            except Exception as error:
                self._pp_messages.put((generation,'error',str(error)))
        threading.Thread(target=worker,daemon=True).start()

    def poll_pick_place(self):
        while True:
            try: generation,kind,value=self._pp_messages.get_nowait()
            except queue.Empty: break
            if generation!=self._pp_generation or self._closed: continue
            if kind=='progress':
                self.feedback('Planning Pick & Place: '+value); continue
            self._pp_planning=False; self.busy=False
            if kind=='error':
                self._pp_preview=None
                self.log('Pick & Place rejected: '+value)
                self.feedback('No move prepared: '+value,'error'); self.refresh_controls(); continue
            plan=value
            try:
                same_inputs=(np.array_equal(plan.pick,[float(e.get()) for e in self.entries]) and
                             plan.seconds==float(self.move_seconds.get()) and plan.settings==self._pp_settings)
            except ValueError: same_inputs=False
            if not self.ready or not self.initialized or not same_inputs or not np.allclose(plan.start,self.positions,atol=.001,rtol=0):
                self.feedback('Inputs or current pose changed during planning; preview again.','error'); continue
            self._pp_preview=plan; self._pp_plot=plan; self._pp_current_label='Preview only'
            grasp=next(s for s in plan.steps if s.label=='Close claw / grasp')
            self.draw(ik.servo_to_q(self.cal,grasp.pose),plan.pick,None)
            total=sum(s.duration_ms+s.pause_ms for s in plan.steps)/1000
            self.feedback(f'Pick & Place ready: {len(plan.steps)} checked commands, about {total:.1f} s. Inspect cup/block settings, then Run.','success')
            self.log(f'Pick & Place preflight complete: pickup {plan.pick.tolist()}, cup [{plan.settings.x}, {plan.settings.y}], rim {plan.settings.rim_z} mm; release {plan.settings.release_xyz.tolist()}. No motion sent.')
            self.refresh_controls()

    def run_pick_place(self):
        if self.busy or self.stop_pending or self.mode!='I' or not self.initialized or not self.ready or self._pp_preview is None:
            return
        plan=self._pp_preview
        try:
            if not np.array_equal(plan.pick,[float(e.get()) for e in self.entries]) or plan.settings!=self._pp_settings or plan.seconds!=float(self.move_seconds.get()):
                raise ValueError('Inputs changed; preview Pick & Place again')
            if not np.allclose(plan.start,self.positions,atol=.001,rtol=0):
                raise ValueError('Arm moved; preview Pick & Place again')
            if not self.supports_timed_motion(): raise ValueError('Timed firmware is not available')
            self._pp_preview=None; self.preview=None; self.plan=[]
            self._pp_steps=list(plan.steps); self._pp_step_index=0; self._pp_expected=list(plan.start)
            self._pp_running=True; self.busy=True; self.active='PICKPLACE'; self.stopped=False
            self.knob_baseline=list(self.knobs) if self.knobs else None
            self.send_pick_place_step()
        except ValueError as error:
            self._pp_preview=None; self.feedback(str(error),'error')
        except Exception as error: self.fail(str(error))
        self.refresh_controls()

    def send_pick_place_step(self):
        self._pp_delay=None
        if not self._pp_running or self._closed: return
        try:
            if self._pp_step_index>=len(self._pp_steps):
                self._pp_running=False; self.busy=False; self.pending=None
                self._pp_current_label='Complete'
                self.log('Pick & Place command sequence complete; visually verify the block landed in the cup.')
                self.feedback('Pick & Place complete: claw open, arm retreated. Grasp/drop are not sensor-verified.','success')
                self.refresh_controls(); return
            if not np.allclose(self._pp_expected,self.positions,atol=.01,rtol=0):
                raise ValueError('Current commands no longer match the checked task; preview again')
            step=self._pp_steps[self._pp_step_index]; self._pp_step_index+=1
            self._pp_current_step=step; self._pp_current_label=step.label
            self.pending=list(step.pose); self._pp_expected=list(step.pose)
            self.started=time.monotonic(); self.active='PICKPLACE'
            self.draw(ik.servo_to_q(self.cal,step.pose),step.xyz,step.result)
            self.log(f'Pick & Place {self._pp_step_index}/{len(self._pp_steps)}: {step.label}')
            self.feedback('Running Pick & Place: '+step.label)
            self.send(f'T {step.duration_ms} '+' '.join(f'{float(v):.6g}' for v in step.pose))
            self.refresh_controls()
        except Exception as error: self.fail(str(error))

    def pick_place_step_done(self):
        if not self._pp_running: return
        pause=self._pp_current_step.pause_ms
        self.started=time.monotonic()
        if pause:
            self._pp_delay=self.root.after(pause,self.send_pick_place_step)
        else: self.send_pick_place_step()

    def calculate(self,auto=False):
        job=getattr(self,'_auto_job',None)
        if job is not None and not auto:      # Enter / button: run now, drop the pending auto run
            try: self.root.after_cancel(job)
            except Exception: pass
            self._auto_job=None
        if self.busy or getattr(self,'stop_pending',False): self.log('Wait for motion/STOP confirmation first.'); return
        self._pp_plot=None
        self.invalidate()
        if not self.cal or not self.initialized:
            self.log('Load calibration and Initialize before calculating.'); return
        if self.mode!='I': self.log('Select IK CONTROL first.'); return
        try:
            values=[]
            for name,entry in zip(('X','Y','Z'),self.entries):
                try:
                    value=float(entry.get())
                    if not np.isfinite(value): raise ValueError
                except ValueError:
                    raise ValueError(f'{name} must be a finite number in millimetres') from None
                values.append(value)
            target=np.array(values)
            start=list(self.positions); q0=ik.servo_to_q(self.cal,start)
            q,r,waypoints,miss,seed_note=self.solve_with_retries(target,start,q0,retries=not auto)
            if waypoints is None:
                if auto and RETRY_SEEDS:
                    self.feedback(f'Auto-preview: no solution from the current pose ({r["status"]}, '
                                  f'{r["error"]*1000:.1f} mm off). Press Calculate / Preview to also try '
                                  'other starting poses.','error')
                else:
                    self.feedback(f'No move prepared: {r["status"]}; position error {r["error"]*1000:.2f} mm '
                                  f'(also tried {len(retry_seeds(self.m,target))} other starting poses). '
                                  'The target is probably out of reach or blocked.','error')
                self.draw(q,target,r); return
            duration=self.chosen_duration(start,waypoints[0])
            self.preview=dict(start=start,target=target,q=q,result=r,waypoints=waypoints,duration_ms=duration)
            self.move_button.config(state='normal')
            retry_text=f' (found by retrying from {seed_note})' if seed_note else ''
            retry_text=(' (auto)' if auto else '')+retry_text
            self.feedback(f'Preview ready{retry_text} — IK error {r["error"]*1000:.3f} mm; rounded-command miss {miss:.2f} mm; move time {duration/1000:.2f} s. Move Arm to execute.','success')
            self.log(f'Angles deg {np.rad2deg(ik.real_angles(q)).round(2).tolist()}; rounded miss {miss:.2f} mm; one checked coordinated target; {duration/1000:.2f} s cubic motion. Preview only.')
            # Show the calibrated/rounded command endpoint, so the solid arm
            # coincides with this ghost when the reported command arrives.
            self.draw(ik.servo_to_q(self.cal,waypoints[0]),target,r)
        except Exception as e:
            message='Cannot prepare move: '+str(e)
            self.log(message); self.feedback(message,'error')
        finally: self.refresh_controls()

    def solve_with_retries(self,target,start,q0,retries=True):
        """IK from the current pose; if that fails, retry from other starting poses.

        The solver is local: from an awkward pose (e.g. arm nearly straight) it
        can pin joints at their limits and stall although the target is reachable.
        A retry only changes the solver's first guess. The move itself is still
        checked as one coordinated path from where the arm actually is now.
        Returns (q, result, waypoints, miss, note); waypoints is None on failure.
        """
        q,r=ik.inverse_kinematics(target,self.m,q0)
        self.log(f'XYZ {target.tolist()} mm from current pose: {r["status"]}; '
                 f'{len(r["history"])-1} updates; residual {r["error"]*1000:.4f} mm')
        first_failure=None
        if r['success']:
            try:
                waypoints,miss=ik.plan_motion(self.m,self.cal,start,target,r,MAX_MISS_MM)
                return q,r,waypoints,miss,''
            except ValueError as e:
                first_failure=str(e)
                self.log('Current-pose solution rejected: '+first_failure)
        if not RETRY_SEEDS or not retries:   # auto-preview skips the slower retries
            if first_failure: raise ValueError(first_failure)
            return q,r,None,None,''
        best=None; failed=(q,r)
        for seed in retry_seeds(self.m,target):
            label='['+', '.join(f'{v:.0f}' for v in np.rad2deg(seed))+'] deg'
            try:
                qs,rs=ik.inverse_kinematics(target,self.m,seed)
            except ValueError as e:
                self.log(f'  retry from {label}: skipped ({e})'); continue
            if not rs['success']:
                self.log(f'  retry from {label}: {rs["status"]}, {rs["error"]*1000:.2f} mm')
                if rs['error']<failed[1]['error']: failed=(qs,rs)
                continue
            # Same answer, but recorded as starting from the arm's real pose so the
            # move is planned and checked from there (plan_motion requires it).
            history=np.vstack([q0,rs['history'][-1]])
            rr=dict(rs,history=history,seed=seed)
            try:
                waypoints,miss=ik.plan_motion(self.m,self.cal,start,target,rr,MAX_MISS_MM)
            except ValueError as e:
                self.log(f'  retry from {label}: solved, but move rejected ({e})'); continue
            change=float(np.abs(np.rad2deg(qs-q0)).sum())
            self.log(f'  retry from {label}: converged in {len(rs["history"])-1} updates; '
                     f'total joint change {change:.0f} deg')
            if best is None or change<best[0]:
                best=(change,qs,rr,waypoints,miss,label)
        if best is None:
            if first_failure: raise ValueError(first_failure+' (retries from other starting poses also failed)')
            return failed[0],failed[1],None,None,''
        _,qs,rr,waypoints,miss,label=best
        self.log(f'Using the solution found from starting pose {label} (smallest joint change).')
        return qs,rr,waypoints,miss,label

    def move(self):
        if self.busy or getattr(self,'stop_pending',False) or self.mode!='I' or not self.initialized or not self.preview: return
        p=self.preview
        try:
            if not np.array_equal(p['target'],[float(e.get()) for e in self.entries]):
                raise ValueError('Target changed; calculate again')
            if self.positions!=p['start']: raise ValueError('Arm moved; calculate again')
            waypoints,_=ik.plan_motion(self.m,self.cal,self.positions,p['target'],p['result'],MAX_MISS_MM)
            duration=self.chosen_duration(self.positions,waypoints[0])
            self.invalidate(); self.launch(waypoints,command='T',duration_ms=duration)
        except Exception as e: self.fail(str(e))

    def grip(self,closed):
        if self.busy or getattr(self,'stop_pending',False) or not self.initialized or self.mode=='K' or not self.cal: return
        self.invalidate(); pose=list(self.positions)
        pose[0]=self.cal.claw_closed if closed else self.cal.claw_open
        try: self.launch([pose],command='T',duration_ms=self.chosen_duration(self.positions,pose))
        except Exception as e: self.fail(str(e))

    def reload(self):
        if self.synthetic:
            self.log('Synthetic simulation ignores calibration.json. Restart with --simulate --use-calibration to test a saved calibration; hardware files have not been changed.')
            return
        if self.busy or getattr(self,'stop_pending',False): self.log('Wait for movement/STOP confirmation before reloading calibration.'); return
        try:
            cal=mc.Calibration.load(calibration_path()); m=ik.model(GEOMETRY,cal)
            self.cal=cal; self.m=m; self.synthetic=False
            self.invalidate(clear_display=True); self.initialized=False; self.update_init_text()
            for j,s in self.sliders.items():
                lo,hi=self.limit(j); s.config(from_=hi,to=lo)
            self.reset_pending()
            if getattr(self.link,'simulated',False):
                self.log('Calibration loaded. Reinitializing emulator upright for IK preview.')
                self.initialize()
            else:
                if getattr(self,'servos_on',False) and self.ready:
                    limits=' '.join(f'{lo} {hi}' for lo,hi in (self.limit(j) for j in range(5)))
                    self.send(f'CFG {limits} {STEP_MS} {SETTLE_MS}')
                    self.log('Calibration loaded. Servos are still on: click "Slowly straighten upright" to return upright with the new calibration.')
                else:
                    self.log('Calibration loaded; servos require explicit initialization. Arrange the arm upright before first enable, then Straighten arm upright.')
        except Exception as e: self.log(str(e))
        finally: self.refresh_controls()

    def redraw_preview(self):
        """Retain the same calculated preview when selecting/resizing a view."""
        self.draw()

    def draw(self,q=None,target=None,result=None):
        """3D arm drawing on self.canvas (red box); error curve on self.error_canvas (purple box)."""
        c=self.canvas; c.delete('all')
        # Only an explicit calculation installs a new target snapshot. Rendering,
        # slider changes and serial reports never overwrite it with the live arm.
        if q is not None:
            saved_q=np.array(q,dtype=float,copy=True)
            saved_target=None if target is None else np.array(target,dtype=float,copy=True)
            saved_result=copy.deepcopy(result)
            points=np.array(ik.fk(self.m,saved_q)[1]*1000,copy=True)
            saved_q.setflags(write=False)
            points.setflags(write=False)
            if saved_target is not None: saved_target.setflags(write=False)
            self._preview_plot=(saved_q,saved_target,saved_result)
            self._target_points=points
        candidate=self._preview_plot is not None
        if candidate:
            q,target,result=self._preview_plot
        self.draw_error(result)
        # APP and IK own different target snapshots. Serial position updates
        # refresh only the solid arm, never either mode's dashed target.
        if self.mode=='A':
            candidate=getattr(self,'_app_target_points',None) is not None
            target=None
        elif self.mode=='K':
            candidate=False
            target=None
        current_q=self.m.q0
        if self.cal:
            try:
                current_q=ik.servo_to_q(self.cal,self.positions if self.initialized else self.start_pose())
            except ValueError:
                pass  # No valid upright reference; initialization reports why.
        current=ik.fk(self.m,current_q)[1]*1000
        if candidate:
            P=self._app_target_points if self.mode=='A' else self._target_points
        else:
            P=current
        width=c.winfo_width(); height=c.winfo_height()
        if width<=1: width=480
        if height<=1: height=360
        if width<260 or height<180:
            c.create_text(width/2,height/2,text='Enlarge the window to show the 3D preview.',fill='#64748b')
            return
        view=self.preview_view.get()
        axes=np.eye(3)*70.
        fit_points=preview_workspace_points(self.m,target)
        task_plot=self._pp_plot if self.mode=='I' else None
        if task_plot is not None:
            s=task_plot.settings; radius=s.diameter/2+5
            extra=[[s.x+dx,s.y+dy,z] for dx in (-radius,radius)
                   for dy in (-radius,radius) for z in (0,s.rim_z)]
            fit_points=np.vstack((fit_points,extra,[step.xyz for step in task_plot.steps]))
        xy=fit_points[:,:2]
        low=xy.min(axis=0)-12; high=xy.max(axis=0)+12
        floor=np.array([[low[0],low[1],0],[high[0],low[1],0],
                        [high[0],high[1],0],[low[0],high[1],0]])
        project=fit_preview_projection(np.vstack((fit_points,floor)),view,
                                       (28,64,width-28,height-44))
        c.create_polygon(*[v for p in floor for v in project(p)],
                         fill='#f1f5f9',outline='#cbd5e1',tags='table_plane')
        for axis in (0,1):
            for value in np.arange(math.ceil(low[axis]/50)*50,high[axis],50):
                a=np.r_[low,0.]; b=np.r_[high,0.]; a[axis]=b[axis]=value
                c.create_line(*project(a),*project(b),fill='#dce4ed')
        if task_plot is not None:
            s=task_plot.settings; radius=s.diameter/2
            rings=[]
            for z in (0.,s.rim_z):
                ring=[np.array([s.x+radius*math.cos(t),s.y+radius*math.sin(t),z])
                      for t in np.linspace(0,2*math.pi,33)]
                rings.append(ring)
                c.create_line(*[v for point in ring for v in project(point)],fill='#0f766e',width=2,tags='cup')
            for i in (0,8,16,24):
                c.create_line(*project(rings[0][i]),*project(rings[1][i]),fill='#0f766e',tags='cup')
            cx,cy=project([s.x,s.y,s.rim_z])
            c.create_text(cx,cy-12,text=f'Cup rim {s.rim_z:g} mm',fill='#0f766e')
            route=[ik.position(self.m,ik.servo_to_q(self.cal,task_plot.start))*1000,
                   *[step.xyz for step in task_plot.steps]]
            c.create_line(*[v for point in route for v in project(point)],fill='#5eead4',width=1,tags='task_tcp_route')
        def arm(points,color,dashed=False,tag='current_link'):
            for a,b in zip(points[:-1],points[1:]):
                if np.linalg.norm(b-a)>1e-9:
                    c.create_line(*project(a),*project(b),fill=color,width=3 if dashed else 4,
                                  dash=(5,3) if dashed else (),tags=tag)
            for p in points:
                x,y=project(p)
                c.create_oval(x-3,y-3,x+3,y+3,fill='white' if dashed else color,outline=color)
        # Draw the requested pose as a ghost; the current estimate stays solid.
        # Current is drawn last so shared/overlapping links remain solid.
        if candidate: arm(P,'#1971c2',True,tag='preview_link')
        arm(current,'#7048a5',tag='current_link')
        for j,(label,color) in enumerate((('X right','#e03131'),('Y forward','#2b8a3e'),('Z up','#1971c2'))):
            a=np.array(project([0,0,0])); b=np.array(project(axes[j])); delta=b-a
            if np.linalg.norm(delta)<1e-8:
                # An axis pointing at the viewer has no projected line length.
                c.create_oval(a[0]-5,a[1]-5,a[0]+5,a[1]+5,outline=color,width=2)
                c.create_oval(a[0]-1,a[1]-1,a[0]+1,a[1]+1,fill=color)
                c.create_text(18,height-42,text=label+' (toward viewer)',anchor='w',fill=color)
            else:
                c.create_line(*a,*b,fill=color,width=2,arrow='last',tags='world_axis_'+label[0].lower())
                dx=6 if delta[0]>=0 else -6
                c.create_text(b[0]+dx,b[1]-8,text=label,anchor='w' if dx>0 else 'e',fill=color)
        a=project([0,0,0]);c.create_text(a[0]-8,a[1]+12,text='{0}',fill='#334155')
        if target is not None:
            x,y=project(target)
            c.create_line(x-6,y-6,x+6,y+6,fill='red',width=2,tags='target_marker')
            c.create_line(x-6,y+6,x+6,y-6,fill='red',width=2,tags='target_marker')
        c.create_text(width*.5,14,text='World frame {0}: X right, Y forward, Z up')
        c.create_text(width*.5,34,text=view,fill='#475569')
        if task_plot is not None:
            c.create_text(width*.5,height-32,text='Teal: cup and TCP stage endpoints (schematic)',fill='#0f766e')
        legend=('Solid purple: current estimate | Dashed blue: slider / sent target' if self.mode=='A' else
                'Solid purple: current estimate | Dashed blue: target pose | Red: XYZ') if candidate else (
            'Purple: current estimated pose' if self.initialized else 'Purple: startup model pose (not initialized)')
        c.create_text(width*.5,height-16,text=legend,fill='#475569')

    def draw_error(self,result=None):
        """Solver convergence: position error per update, log scale (IK panel)."""
        c=getattr(self,'error_canvas',None)
        if c is None: return
        c.delete('all')
        width=c.winfo_width(); height=c.winfo_height()
        if width<=1: width=360
        if height<=1: height=280
        if self._pp_plot is not None and self.mode=='I':
            plan=self._pp_plot
            c.create_text(width*.5,15,text='Pick & Place - checked command sequence',fill='#0f766e')
            labels=[]
            for step in plan.steps:
                label=step.label.split(' (')[0]
                if not labels or labels[-1]!=label: labels.append(label)
            for i,label in enumerate(labels):
                c.create_text(14,43+i*22,text=f'{i+1}. {label}',anchor='w',fill='#334155')
            total=sum(s.duration_ms+s.pause_ms for s in plan.steps)/1000
            c.create_text(width*.5,height-35,text=f'{len(plan.steps)} commands; about {total:.1f} s')
            c.create_text(width*.5,height-15,text=self._pp_current_label,fill='#0f766e')
            return
        c.create_text(width*.5,14,text='Position error (mm), logarithmic scale')
        if result is None:
            c.create_text(width*.5,height*.5,text='Calculate / Preview to show convergence',fill='#64748b')
            return
        errors=np.maximum(result['errors']*1000,1e-6); logs=np.log10(errors)
        lo=math.floor(min(-1.,float(logs.min()))); hi=math.ceil(max(0.,float(logs.max())))
        left=58; right=width-18; top=40; bottom=height-50
        for power in range(lo,hi+1,max(1,math.ceil((hi-lo)/6))):
            y=bottom-(bottom-top)*(power-lo)/(hi-lo)
            c.create_line(left,y,right,y,fill='#e2e8f0')
            c.create_text(left-6,y,text=f'{10.**power:g}',anchor='e',fill='#64748b')
        coords=[]
        for i,v in enumerate(logs):coords.extend((left+(right-left)*i/max(1,len(logs)-1),bottom-(bottom-top)*(v-lo)/(hi-lo)))
        if len(coords)>2: c.create_line(*coords,fill='#1971c2',width=2)
        c.create_text(left,bottom+14,text='0',fill='#64748b')
        c.create_text(right,bottom+14,text=str(len(errors)-1),fill='#64748b')
        c.create_text(width*.5,bottom+14,text='solver update',fill='#94a3b8')
        c.create_text(width*.5,height-14,text=f'{result["status"]}: {len(errors)-1} updates; final {errors[-1]:.4f} mm')

    # =========================================================================
    # APP CONTROL: sliders, +/- buttons, typed values, per-servo and all-servo send
    # A changed value is "pending" (green box) until it is sent.
    # =========================================================================
    def capture_app_target(self,pose):
        """Freeze an APP endpoint; call only on user edits or explicit sends.

        Never rebuild it from serial reports: sliders can follow the live arm
        during a move, while this independent target must remain stationary.
        """
        if not self.cal or not self.initialized:
            self._app_target_pose=None; self._app_target_points=None
            return
        try:
            saved=tuple(float(value) for value in pose)
            q=ik.servo_to_q(self.cal,saved)
            points=np.array(ik.fk(self.m,q)[1]*1000,copy=True)
        except ValueError:
            self._app_target_pose=None; self._app_target_points=None
            return
        points.setflags(write=False)
        self._app_target_pose=saved
        self._app_target_points=points

    def _set_slider(self,j,value):
        """Move slider j and its value box without marking it as a user change."""
        # Tk Scale.set may invoke its command after _syncing is reset. Remember
        # the expected callback value as well as guarding synchronous callbacks.
        expected=getattr(self,'_programmatic_slider_values',None)
        if expected is None:
            expected={}; self._programmatic_slider_values=expected
        slider=self.sliders[j]
        previous_state=slider.cget('state')
        self._syncing=True
        try:
            # Tk Scale.set ignores changes while disabled (as in IK/KNOB).
            # Enable only for this programmatic write, then restore the lock
            # before returning to Tk's event loop. This sends no servo command.
            if previous_state=='disabled': slider.config(state='normal')
            slider.set(value)
            expected[j]=float(slider.get())  # Scale may round to its resolution
            values=getattr(self,'servo_values',None)
            if values: values[j].set(f'{float(value):.0f}')
        finally:
            if previous_state=='disabled': slider.config(state=previous_state)
            self._syncing=False

    def _mark(self,j):
        """Pending if the slider differs from the servo's current command."""
        try: changed=round(float(self.sliders[j].get()))!=round(float(self.positions[j]))
        except (TypeError,ValueError): changed=False
        pending=getattr(self,'pending_servos',None)
        if pending is None: return
        if changed: pending.add(j)
        else: pending.discard(j)
        entries=getattr(self,'servo_entries',None)
        if entries: entries[j].config(bg=COLORS['pending'] if j in pending else 'white')
        self.update_send_colors()
        if self.mode=='A' and not self.busy:
            # Untouched joints start at the latest reported commands, not at
            # old or rounded slider values. Only pending edits change the pose.
            pose=self.unsent_pose()
            self.capture_app_target(self.positions if pose is None else pose)
            if hasattr(self,'canvas'): self.redraw_preview()

    def update_send_colors(self):
        """Send buttons are light green only when pressing them would send a change."""
        face=getattr(self,'button_face',None)
        if face is None: return
        pending=getattr(self,'pending_servos',set())
        for j,button in self.send_buttons.items():
            button.config(bg=COLORS['send'] if j in pending else face)
        self.manual_button.config(bg=COLORS['send'] if pending else face)

    def unsent_pose(self):
        """Servo pose if every unsent slider were sent now (None if nothing is unsent)."""
        pending=getattr(self,'pending_servos',None)
        if not pending or not getattr(self,'sliders',None): return None
        pose=list(self.positions)
        for j in pending: pose[j]=float(self.sliders[j].get())
        return pose

    def _slider_moved(self,j):
        if getattr(self,'_syncing',False): return
        expected=getattr(self,'_programmatic_slider_values',{}).pop(j,None)
        if expected is not None and abs(float(self.sliders[j].get())-expected)<1e-6:
            return
        if self.mode!='A' or not self.initialized or self.busy or getattr(self,'stop_pending',False): return
        values=getattr(self,'servo_values',None)
        if values: values[j].set(f'{float(self.sliders[j].get()):.0f}')
        self._mark(j)

    def _clamped(self,j,value):
        lo,hi=self.limit(j)
        return int(min(max(round(value),lo),hi))

    def nudge(self,j,step):
        if self.mode!='A' or not self.initialized: return
        self.apply_typed(j,quiet=True)
        value=self._clamped(j,float(self.sliders[j].get())+step)
        self._set_slider(j,value); self._mark(j)

    def apply_typed(self,j,quiet=False):
        """Typed value -> slider (kept inside the safe range). Never sends."""
        values=getattr(self,'servo_values',None)
        if not values: return
        text=values[j].get().strip()
        try:
            typed=float(text)
            if not np.isfinite(typed): raise ValueError
        except ValueError:
            self._set_slider(j,float(self.sliders[j].get())); self._mark(j)
            if text and not quiet: self.feedback(f'{JOINT_NAMES[j]}: "{text}" is not a number; value reset.','error')
            return
        value=self._clamped(j,typed)
        if value!=typed and not quiet:
            lo,hi=self.limit(j)
            self.feedback(f'{JOINT_NAMES[j]}: {typed:g} is outside the safe range {lo}..{hi}; set to {value}.','info')
        self._set_slider(j,value); self._mark(j)

    def _manual_allowed(self):
        if self.busy or getattr(self,'stop_pending',False) or not self.initialized or self.mode!='A':
            self.log('Manual controls require initialized APP mode and no active move.'); return False
        return True

    def _send_manual_pose(self,pose,joints):
        """Send a manual pose: smooth timed move when the firmware supports it."""
        # A per-servo Send previews exactly that command; Send all previews all
        # five. Other unsent edits stay green but are not extra ghost arms.
        self.capture_app_target(pose)
        self.invalidate()
        try:
            if self.supports_timed_motion():
                self.launch([pose],command='T',duration_ms=self.chosen_duration(self.positions,pose))
            else:
                self.launch([pose])
            pending=getattr(self,'pending_servos',set())
            for j in joints: pending.discard(j)
            for j in joints:
                entries=getattr(self,'servo_entries',None)
                if entries: entries[j].config(bg='white')
            self.update_send_colors()
        except Exception as e: self.fail(str(e))

    def send_servo(self,j):
        """Move ONLY servo j to its slider value; the other four stay where they are."""
        if not self._manual_allowed(): return
        self.apply_typed(j,quiet=True)
        pose=list(self.positions); pose[j]=int(float(self.sliders[j].get()))
        self.log(f'Send {JOINT_NAMES[j]} (S{j+1}) -> {pose[j]}')
        self._send_manual_pose(pose,[j])

    def manual(self):
        """Send all five sliders together (the original 'Send manual sliders')."""
        if not self._manual_allowed(): return
        for j in range(5): self.apply_typed(j,quiet=True)
        pose=[int(float(self.sliders[j].get())) for j in range(5)]
        self.log('Send all sliders: S1..S5 '+' '.join(map(str,pose)))
        self._send_manual_pose(pose,list(range(5)))

    def reset_pending(self):
        """Forget unsent slider changes; sliders snap back to the servo commands."""
        pending=getattr(self,'pending_servos',None)
        if pending: pending.clear()
        for j in getattr(self,'sliders',{}):
            try: self._set_slider(j,self.positions[j])
            except Exception: pass
            entries=getattr(self,'servo_entries',None)
            if entries: entries[j].config(bg='white')
        self.update_send_colors()
        self._app_target_pose=None; self._app_target_points=None
        if hasattr(self,'canvas'): self.redraw_preview()

    # =========================================================================
    # KNOB CONTROL display
    # =========================================================================
    def knob_servo_value(self,j,raw):
        """Same mapping as the Arduino sketch: knob 0..1023 -> safe servo range."""
        lo,hi=self.limit(j)
        return lo+(int(raw)*(hi-lo)+511)//1023

    def toggle_knob_units(self,j):
        self.knob_show_raw[j]=not self.knob_show_raw[j]
        self.update_knob_display()

    def update_knob_display(self):
        widgets=getattr(self,'knob_widgets',None)
        if not widgets: return
        knobs=getattr(self,'knobs',None)
        for j,(canvas,needle,text,button) in widgets.items():
            if j is None: continue
            raw_mode=self.knob_show_raw.get(j,False)
            button.config(text=f'{JOINT_NAMES[j]} {"RAW" if raw_mode else "Value"}')
            if not knobs:
                canvas.itemconfigure(text,text='--'); canvas.itemconfigure(needle,state='hidden'); continue
            raw=knobs[j]
            canvas.itemconfigure(text,text=str(raw) if raw_mode else str(self.knob_servo_value(j,raw)))
            angle=math.radians(45+270*min(max(raw,0),1023)/1023)     # knob sweep, flipped 180 deg to match the real knobs
            canvas.coords(needle,60+34*math.sin(angle),60-34*math.cos(angle),60+48*math.sin(angle),60-48*math.cos(angle))
            canvas.itemconfigure(needle,state='normal')
        if hasattr(self,'knob_note'):
            self.knob_note.set('Value = servo command the knob asks for; RAW = knob reading 0-1023.'
                               if knobs else 'No knob readings yet (the simulator has no knobs).')

    # =========================================================================
    # IK claw toggle
    # =========================================================================
    def claw_is_closed(self):
        if not self.cal: return False
        claw=float(self.positions[CLAW])
        return abs(claw-self.cal.claw_closed)<abs(claw-self.cal.claw_open)

    def toggle_claw(self):
        self.grip(not self.claw_is_closed())

    # =========================================================================
    # Readouts
    # =========================================================================
    def update_servo_line(self):
        if hasattr(self,'servo_line'):
            self.servo_line.set('Actual Servo Values   '+'   '.join(
                f'S{j+1} {JOINT_NAMES[j]} {float(self.positions[j]):.0f}' for j in range(5)))

    def update_init_text(self):
        self.init_button.config(text=self.upright_button_text())
        try:
            pose=self.start_pose()
            values='['+', '.join(f'{float(v):g}' for v in pose)+']'
            reference='Upright DH: q=[90,90,0,0] deg' if self.cal else 'UNCALIBRATED: angles unknown'
            self.init_text.set(f'{reference}\nServos 1-5 {values}')
        except ValueError as e:
            self.init_text.set('Upright unavailable: '+str(e))

    def update_position(self):
        pending=getattr(self,'pending_servos',set())
        for j,s in self.sliders.items():
            if j in pending: continue                      # keep unsent changes
            if hasattr(self,'servo_values'): self._set_slider(j,self.positions[j])
            else: s.set(self.positions[j])
        if self.cal:
            q=ik.servo_to_q(self.cal,self.positions)
            xyz=ik.position(self.m,q)*1000
            self.readout.set('Current estimate   XYZ mm: '+np.array2string(xyz,precision=1,suppress_small=True)+
                '\nbase/shoulder/elbow/wrist deg: '+np.array2string(np.rad2deg(ik.real_angles(q)),precision=1,suppress_small=True))
        else: self.readout.set('Raw S1..S5: '+str(self.positions)+' | calibration needed for XYZ')
        self.update_servo_line()
        if self.preview and self.positions!=self.preview['start']: self.invalidate()
        if self._pp_preview is not None and not np.allclose(self.positions,self._pp_preview.start,atol=.001,rtol=0):
            self._pp_preview=None
            self.feedback('Arm moved since Pick & Place preview; preview the task again.')
        self.redraw_preview()  # retain the last calculated pose

    def save_log(self):
        from tkinter import filedialog
        path=filedialog.asksaveasfilename(defaultextension='.txt',initialfile='Lab03_experiment_log.txt')
        if path:
            with open(path,'w',encoding='utf-8') as f: f.write(self.terminal.get('1.0','end'))
            self.log('Log saved: '+path)

    def close(self):
        if getattr(self,'_closed',False): return
        self._closed=True
        for name in ('_poll_id','_auto_job'):
            job=getattr(self,name,None)
            if job is not None:
                try: self.root.after_cancel(job)
                except tk.TclError: pass
                setattr(self,name,None)
        try:
            self.stop()
        finally:
            if self.link: self.link.close()
            self.root.destroy()

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description='miniArm XYZ GUI; explicit offline mode')
    p.add_argument('--simulate',action='store_true',help='Offline emulator with synthetic calibration; ignores local calibration.json')
    p.add_argument('--use-calibration',action='store_true',help='With --simulate, explicitly load and test the saved hardware calibration')
    p.add_argument('--port',default=SERIAL_PORT)
    args=p.parse_args()
    root=tk.Tk()
    try:
        app=ArmApp(root,simulate=args.simulate,port=args.port,use_calibration=args.use_calibration)
    except mc.CalibrationError as e:
        root.destroy(); p.error(str(e))
    root.mainloop()

