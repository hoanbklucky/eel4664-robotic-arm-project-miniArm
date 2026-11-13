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
    "L3": 85.0,   # wrist axis to the tool point (middle of the claw fingers)
}

# --- Behaviour ---
STEP_MS = 15               # ms per servo unit while moving (bigger = slower, smoother)
SETTLE_MS = 0            # IK CONTROL moves one joint at a time; pause between joints (ms)
KNOB_TAKEOVER_DEG = 10     # turning a knob this far switches to KNOB CONTROL
MAX_MISS_MM = 10           # maximum predicted XYZ miss after calibration/rounding

# =============================================================================
# END OF SETUP
# =============================================================================

import numpy as np
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import scrolledtext

import miniarm_calibration as mc

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
DISPLAY_ORDER = [BASE, ARM, FOREARM, WRIST, CLAW]   # left to right on screen: bottom of the arm first

MODES = {   # code sent to the Arduino: (button text, color)
    "A": ("APP CONTROL", "#2b8a3e"),
    "K": ("KNOB CONTROL", "#e8590c"),
    "I": ("IK CONTROL", "#1c7ed6"),
}

# Camera locations in the SAME world frame as FK: X right, Y forward, Z up.
# Rear means the Arduino/knob side (negative Y). These are views, not new frames.
PREVIEW_CAMERAS = {
    'Rear right (Arduino side)': (1., -1., .8),
    'Rear (Arduino side)': (0., -1., .7),
    'Top': (0., 0., 1.),
    'Right side': (1., 0., 0.),
}


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


def calibration_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), CALIBRATION_FILE)


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

    def __init__(self, port, baud, inbox):
        self.port = serial.Serial(port, baud, timeout=0.2)   # this resets the Uno
        self.port.reset_input_buffer()             # throw away anything from before the reset
        self.inbox = inbox
        threading.Thread(target=self._read_forever, daemon=True).start()

    def _read_forever(self):
        while True:
            try:
                raw = self.port.readline()
            except Exception as e:                 # cable pulled, port closed
                self.inbox.put(f"ERR serial: {e}")
                return
            line = clean_line(raw)
            if line:
                self.inbox.put(line)

    def send(self, line):
        self.port.write((line + "\n").encode())

    def tick(self):
        pass

    def close(self):
        self.port.close()


class SimulatedArm:
    """Pretends to be miniarm_controlC.ino so the GUI works without hardware.
    Same commands and replies, except there are no knob readings."""

    simulated = True
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
        self.last_step = self.last_report = 0.0
        self.changed = False
        self.step_ms,self.settle_ms=STEP_MS,SETTLE_MS
        inbox.put("READY")

    def send(self, line):
        parts = line.split()
        cmd, nums = parts[0], [int(p) for p in parts[1:] if p.lstrip("-").isdigit()]
        clamp = lambda i, v: max(self.limits[i][0], min(self.limits[i][1], v))
        if cmd == "CFG" and len(nums) == 12:
            self.limits = [(max(0,min(180,min(nums[2*i:2*i+2]))),
                            max(0,min(180,max(nums[2*i:2*i+2])))) for i in range(5)]
            self.step_ms,self.settle_ms=max(1,nums[10]),max(0,nums[11])
            self.inbox.put("CFG OK")
        elif cmd == "INIT" and len(nums) == 5:
            self.seq,self.move_active=None,False  # INIT freezes old motion, as firmware does
            self.cur = [clamp(i, v) for i, v in enumerate(nums)]
            self.tgt = list(self.cur)
            self.on = self.changed = True
            self.inbox.put("OK")
        elif cmd == "MODE":
            if len(parts)!=2 or parts[1] not in ('A','K','I'):
                self.inbox.put('ERR MODE is A, K or I'); return
            self.tgt, self.move_active, self.seq = list(self.cur), False, None
            self.mode = parts[1]
            self.inbox.put(f"MODE {self.mode}")
        elif cmd in ("M", "J") and len(nums) == 5:
            if not self.on:
                self.inbox.put('ERR send INIT first'); return
            if self.mode == "K":
                self.inbox.put("ERR in knob mode")
                return
            self.tgt, self.seq, self.move_active = list(self.cur), None, False
            targets = [clamp(i, v) for i, v in enumerate(nums)]
            if cmd == "M":
                self.tgt, self.move_active = targets, True
            else:
                self.seq = [targets, 0, 0.0]
        elif cmd == "STOP":
            self.tgt, self.move_active, self.seq = list(self.cur), False, None
            self.inbox.put("STOPPED")
            self.inbox.put("P " + " ".join(str(v) for v in self.cur))

    def _step(self, i, target):
        if self.cur[i] != target:
            self.cur[i] += 1 if self.cur[i] < target else -1
            self.changed = True
        return self.cur[i] == target

    def tick(self):
        now = time.monotonic()
        while self.on and now - self.last_step >= self.step_ms / 1000:
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
        self.root=root; root.title('miniArm 3D XYZ Control')
        self.inbox=queue.Queue(); self.link=None; self.ready=False
        self.mode='A'; self.positions=[90]*5; self.commanded=[90]*5
        self.initialized=False; self.busy=False; self.pending=None
        self.preview=None; self.plan=[]; self.active=None; self.started=0
        self._preview_plot=None
        self.knobs=None; self.knob_baseline=None; self.synthetic=False
        self.stopped=False; self.stop_pending=False; self.stop_acknowledged=False; self.knob_takeover_pending=False
        self.cal,self.synthetic=load_startup_calibration(simulate,use_calibration)
        self.m=ik.model(GEOMETRY,self.cal)
        self._build()
        if simulate:
            self.link=SimulatedArm(self.inbox)
        elif serial is None:
            self.log('pyserial missing. Install requirements or use --simulate.')
        else:
            try: self.link=SerialLink(port,BAUD_RATE,self.inbox)
            except Exception as e: self.log(f'Connection failed: {e}. Use --simulate for offline work.')
        self.status.set(('SIMULATION: synthetic calibration; no physical movement' if self.synthetic else
                         'SIMULATION: saved hardware calibration; no physical movement')
                        if simulate else f'Hardware port: {port}')
        if self.synthetic:
            self.log('SIMULATION uses synthetic calibration; local calibration.json is ignored. Use --simulate --use-calibration to test your measured mapping.')
        elif self.cal:
            self.log(f'Saved calibration loaded: {self.cal.angle_convention} angle convention. Check angle references if IK limits seem wrong.')
        if not simulate:
            self.log('Connection leaves servos off. With calibration, arrange the unloaded arm upright before first enable; the upright command ignores sliders.')
        self.poll()  # process the emulator READY now; subsequent polls schedule themselves
        root.protocol('WM_DELETE_WINDOW',self.close)

    def _build(self):
        bar=tk.Frame(self.root); bar.pack(fill='x',padx=10,pady=8)
        self.mode_buttons={}
        for mode in ('A','K','I'):
            button=tk.Button(bar,text=MODES[mode][0],command=lambda m=mode:self.set_mode(m))
            button.pack(side='left',padx=3); self.mode_buttons[mode]=button
        self.stop_button=tk.Button(bar,text='STOP',bg='#c92a2a',fg='white',command=self.stop)
        self.stop_button.pack(side='left',padx=6)
        self.reload_button=tk.Button(bar,text='Reload calibration',command=self.reload)
        self.reload_button.pack(side='left')
        self.mode_status=tk.StringVar(value='Not initialized')
        tk.Label(self.root,textvariable=self.mode_status,font=('Segoe UI',10,'bold')).pack()
        self.status=tk.StringVar(); tk.Label(self.root,textvariable=self.status).pack()
        setup=tk.Frame(self.root); setup.pack(fill='x',padx=12)
        self.init_text=tk.StringVar(); tk.Label(setup,textvariable=self.init_text).pack(side='left')
        self.init_button=tk.Button(setup,command=self.initialize)
        self.init_button.pack(side='right')
        self.update_init_text()
        panel=tk.LabelFrame(self.root,text='Manual servo commands — raw values, not DH joint angles',padx=12,pady=6)
        panel.pack(padx=10,pady=6)
        self.sliders={}
        for col,j in enumerate(DISPLAY_ORDER):
            box=tk.Frame(panel); box.grid(row=0,column=col,padx=7)
            lo,hi=self.limit(j)
            s=tk.Scale(box,from_=hi,to=lo,length=140,label=JOINT_NAMES[j])
            s.set(90); s.pack(); self.sliders[j]=s
        self.manual_button=tk.Button(panel,text='Send manual sliders (APP mode)',command=self.manual)
        self.manual_button.grid(row=1,column=0,columnspan=5)
        inp=tk.LabelFrame(self.root,text='IK target — XYZ in millimetres',padx=10,pady=6); inp.pack(pady=6)
        self.entries=[]; self.target_values=[]
        for col,(name,value) in enumerate(zip(('X right (mm)','Y forward (mm)','Z up (mm)'),(-40,100,150))):
            tk.Label(inp,text=name).grid(row=0,column=col,padx=10)
            variable=tk.StringVar(value=str(value)); self.target_values.append(variable)
            e=tk.Entry(inp,width=12,textvariable=variable); e.grid(row=1,column=col,padx=10)
            variable.trace_add('write',lambda *args:self.invalidate())
            e.bind('<Return>',lambda event:self.calculate()); self.entries.append(e)
        actions=tk.Frame(self.root); actions.pack()
        self.calculate_button=tk.Button(actions,text='Calculate / Preview',command=self.calculate)
        self.calculate_button.pack(side='left',padx=4)
        self.move_button=tk.Button(actions,text='Move Arm',state='disabled',command=self.move)
        self.move_button.pack(side='left',padx=4)
        self.grip_buttons=[]
        for text,closed in [('Open claw',False),('Close claw',True)]:
            button=tk.Button(actions,text=text,command=lambda c=closed:self.grip(c))
            button.pack(side='left',padx=4); self.grip_buttons.append(button)
        self.save_button=tk.Button(actions,text='Save experiment log',command=self.save_log)
        self.save_button.pack(side='left',padx=4)
        self.action_hint=tk.StringVar()
        tk.Label(self.root,textvariable=self.action_hint,wraplength=1000,justify='left',fg='#475569').pack(padx=12,pady=(5,0))
        self.result_status=tk.StringVar(value='No IK preview calculated.')
        self.result_label=tk.Label(self.root,textvariable=self.result_status,wraplength=1000,justify='left',font=('Segoe UI',9,'bold'))
        self.result_label.pack(padx=12,pady=(3,0))
        self.readout=tk.StringVar(value='Position estimate: not initialized')
        tk.Label(self.root,textvariable=self.readout,font=('Consolas',9)).pack(pady=5)
        views=tk.Frame(self.root); views.pack(fill='x',padx=12)
        tk.Label(views,text='Preview camera:').pack(side='left')
        self.preview_view=tk.StringVar(value='Rear right (Arduino side)')
        tk.OptionMenu(views,self.preview_view,*PREVIEW_CAMERAS,
                      command=lambda _:self.redraw_preview()).pack(side='left')
        tk.Label(views,text='Changing the camera does not move the arm.').pack(side='left',padx=10)
        self.canvas=tk.Canvas(self.root,width=840,height=340,bg='white')
        self.canvas.pack(padx=10,fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda _:self.redraw_preview())
        self.terminal=scrolledtext.ScrolledText(self.root,width=100,height=5,state='disabled',font=('Consolas',9))
        self.terminal.pack(padx=10,pady=8)
        self.draw()
        self.refresh_controls()

    def refresh_controls(self):
        """One place for mode highlighting and initialization/motion gating."""
        waiting=getattr(self,'stop_pending',False)
        idle=self.ready and not self.busy and not waiting
        enabled=idle and self.initialized
        def state(widget,allow):
            if widget is not None and hasattr(widget,'config'): widget.config(state='normal' if allow else 'disabled')
        for mode,button in getattr(self,'mode_buttons',{}).items():
            selected=self.initialized and mode==self.mode
            color=MODES[mode][1] if selected else '#e5e7eb'
            button.config(bg=color,activebackground=color,fg='white' if selected else '#334155',
                          relief='sunken' if selected else 'raised',
                          font=('Segoe UI',9,'bold' if selected else 'normal'))
            state(button,enabled)
        state(getattr(self,'init_button',None),idle and not (self.initialized and self.mode=='K'))
        state(getattr(self,'manual_button',None),enabled and self.mode=='A')
        state(getattr(self,'calculate_button',None),enabled and self.mode=='I' and self.cal is not None)
        state(getattr(self,'move_button',None),enabled and self.mode=='I' and self.preview is not None)
        for button in getattr(self,'grip_buttons',[]):state(button,enabled and self.mode!='K' and self.cal is not None)
        state(getattr(self,'save_button',None),enabled)
        state(getattr(self,'reload_button',None),not self.busy and not waiting)
        for slider in getattr(self,'sliders',{}).values():state(slider,enabled and self.mode=='A')
        for entry in getattr(self,'entries',[]):state(entry,enabled and self.mode=='I')
        if hasattr(self,'mode_status'):
            if waiting: text='STOP requested — awaiting Arduino confirmation'
            elif not self.ready: text='Waiting for connection'
            elif not self.initialized:
                text='Initializing — waiting for controller acknowledgement' if self.busy else 'Not initialized — use the startup button to enable controls'
            else:
                suffix=' — STOPPED (servos holding)' if getattr(self,'stopped',False) else (' — moving' if self.busy else '')
                text='Active mode: '+MODES[self.mode][0]+suffix
            self.mode_status.set(text)
        if hasattr(self,'action_hint'):
            if waiting: hint='Waiting for STOP confirmation and the command-position report.'
            elif not self.ready: hint='Connect the Arduino, or launch with --simulate for offline work.'
            elif not self.initialized:
                hint='Initialization is in progress; wait for the controller acknowledgement.' if self.busy else 'Follow the first-enable procedure, then use the startup button to unlock controls.'
            elif self.busy: hint=f'Moving: {len(getattr(self,"plan",[]))} waypoint(s) queued after the active command. STOP cancels the remaining plan.'
            elif self.mode=='K': hint='Knobs drive the arm. Choose APP or IK before requesting an upright move.'
            elif self.mode=='A': hint='Adjust raw servo sliders, then Send manual sliders. Choose IK to enter an XYZ target.'
            elif self.preview is not None: hint='Preview is ready. Move Arm executes it; editing XYZ requires a new calculation.'
            else: hint='Enter XYZ in mm, then Calculate / Preview (or press Enter). This does not move the arm.'
            self.action_hint.set(hint)

    def feedback(self,message,kind='info'):
        self.feedback_message=message;self.feedback_kind=kind
        if hasattr(self,'result_status'): self.result_status.set(message)
        if hasattr(self,'result_label'):
            self.result_label.config(fg={'info':'#475569','success':'#15803d','error':'#b91c1c'}[kind])

    def log(self,text):
        self.terminal.config(state='normal'); self.terminal.insert('end',text+'\n')
        self.terminal.see('end'); self.terminal.config(state='disabled')

    def limit(self,j):
        return self.cal.limits(j) if self.cal else (0,180)

    def update_init_text(self):
        self.init_button.config(text='Straighten arm upright' if self.cal else 'Enable raw neutral (uncalibrated)')
        try:
            pose=self.start_pose()
            reference='Upright q=[90,90,0,0] deg' if self.cal else 'UNCALIBRATED: angles unknown'
            self.init_text.set(reference+'; S1..S5: '+' '.join(map(str,pose)))
        except ValueError as e:
            self.init_text.set('Upright unavailable: '+str(e))

    def start_pose(self):
        """Fixed upright reference, independent of model seed and manual sliders.

        Without calibration only raw neutral is known, not a physical posture.
        Never clip an unavailable upright pose into a different configuration.
        """
        if self.cal:
            q=np.deg2rad([90.,90.,0.,0.])
            if np.any(q < self.m.limits[:,0]) or np.any(q > self.m.limits[:,1]):
                raise ValueError('upright angles are outside calibrated limits')
            pose=ik.q_to_servo(self.cal,q,self.cal.claw_open)
            if ik.clearance(self.m,ik.servo_to_q(self.cal,pose))[0] <= 0:
                raise ValueError('upright pose violates configured clearance')
            return pose
        return [90]*5

    def send(self,line):
        if not self.ready or not self.link: raise ValueError('Arduino is not ready')
        self.link.send(line)

    def invalidate(self):
        self.preview=None; self.move_button.config(state='disabled')
        self.feedback('Preview cleared — calculate again before Move Arm.')
        self._preview_plot=None
        if hasattr(self,'canvas'): self.draw()
        self.refresh_controls()

    def set_mode(self,mode):
        if not self.ready or not self.initialized or getattr(self,'stop_pending',False):
            self.log('Initialize the arm before selecting a mode.'); return
        if self.busy:
            self.log('Press STOP before changing mode.'); return
        self.invalidate(); self.mode=mode; self.stopped=False
        self.knob_baseline=list(self.knobs) if self.knobs else None
        if self.ready:
            try: self.send('MODE '+mode)
            except Exception as e: self.log(str(e))
        self.log('Mode: '+MODES[mode][0]); self.refresh_controls()

    def initialize(self):
        if not self.ready or self.busy or getattr(self,'stop_pending',False): return
        if self.initialized and self.mode=='K':
            message='Choose APP or IK before an upright move; knob mode would move away from upright.'
            self.log(message); self.feedback(message); return
        try:
            pose=self.start_pose()  # fixed reference; never read manual sliders
            if self.cal:
                self.log('Upright calibration conversion: '+', '.join(
                    f'{JOINT_NAMES[mc.SERVO_INDEX[name]]} q={q:g} deg -> S{mc.SERVO_INDEX[name]+1} raw {pose[mc.SERVO_INDEX[name]]}'
                    for name,q in zip(mc.JOINTS,[90,90,0,0])))
            if self.initialized:
                pose[CLAW]=self.positions[CLAW]  # do not release a held object
                if self.cal:
                    ik.validate_servo_leg(self.m,self.cal,self.positions,pose)
                self.invalidate()
                self.log('Returning upright; keeping '+MODES[self.mode][0]+'.')
                self.launch([pose])  # checked, gradual J move, not another INIT
                return
            self.invalidate()
            limits=' '.join(f'{lo} {hi}' for lo,hi in (self.limit(j) for j in range(5)))
            self.send(f'CFG {limits} {STEP_MS} {SETTLE_MS}')
            self.mode='A'; self.send('MODE A')
            if getattr(getattr(self,'link',None),'simulated',False):
                self.log('Initializing emulator at upright reference; no physical movement.')
            else:
                self.log('First enable: INIT attaches immediately. Physical pose must already be near the displayed reference; no encoder feedback.')
            self.launch([pose],command='INIT')
        except Exception as e:
            message='Cannot straighten/enable arm: '+str(e)
            self.log(message); self.feedback(message,'error')

    def launch(self,poses,command='J'):
        self.stopped=False
        self.knob_baseline=list(self.knobs) if getattr(self,'knobs',None) else None
        self.plan=[list(p) for p in poses]; self.active=command
        self.busy=True; self.refresh_controls(); self.next_waypoint()

    def next_waypoint(self):
        if not self.plan:
            self.busy=False; self.pending=None
            self.log('Command sequence finished (estimated position; no encoder feedback).'); self.refresh_controls(); return
        self.pending=self.plan.pop(0); self.started=time.monotonic()
        self.send(self.active+' '+' '.join(map(str,self.pending)))

    def poll(self):
        if self.link: self.link.tick()
        while not self.inbox.empty():
            line=self.inbox.get(); parts=line.split()
            if not parts: continue
            if line.endswith('READY'):
                self.ready=True; self.initialized=False; self.busy=False
                self.stopped=False; self.stop_pending=False; self.stop_acknowledged=False; self.knob_takeover_pending=False
                self.plan=[]; self.pending=None; self.invalidate()
                if getattr(self.link,'simulated',False):
                    self.log('Emulator READY. Preparing upright reference for direct IK preview.')
                    self.initialize()
                else:
                    self.log('Arduino READY. Servos off; initialize explicitly.')
            elif parts[0]=='P' and len(parts)==6:
                self.positions=[int(v) for v in parts[1:]]; self.update_position()
                if getattr(self,'stop_pending',False) and self.stop_acknowledged:
                    self.stop_pending=False
                    self.log('STOP confirmed: command steps frozen; servos remain powered to hold position.')
                    self.feedback('STOP confirmed — commands frozen, servos holding. Calculate again before an IK move.')
                    if getattr(self,'knob_takeover_pending',False):
                        self.knob_takeover_pending=False
                        self.set_mode('K')
            elif line=='STOPPED':
                self.stop_acknowledged=True
            elif parts[0]=='MODE' and len(parts)==2 and parts[1] in MODES:
                self.mode=parts[1]
            elif line=='OK' and self.pending is not None:
                self.positions=list(self.pending); self.commanded=list(self.pending)
                if self.active=='INIT': self.initialized=True
                self.pending=None; self.update_position()
                try:
                    self.next_waypoint()
                    if self.active=='INIT' and not self.busy and getattr(self.link,'simulated',False):
                        self.set_mode('I')
                        self.log('Simulation ready for XYZ Calculate / Preview; no upright click required.')
                except Exception as e: self.fail(str(e))
            elif parts[0]=='K' and len(parts)==6:
                self.knobs=[int(v) for v in parts[1:]]
                if self.knob_baseline is None: self.knob_baseline=list(self.knobs)
                threshold=KNOB_TAKEOVER_DEG/180*1023
                if self.initialized and not getattr(self,'stopped',False) and self.mode!='K' and any(abs(a-b)>threshold for a,b in zip(self.knobs,self.knob_baseline)):
                    self.stop(); self.knob_takeover_pending=True
                    self.log('Knob takeover: cancelling IK; knob mode starts after STOP confirmation.')
            elif parts[0]=='ERR': self.fail(line)
        if self.busy and time.monotonic()-self.started>30:
            self.stop(); self.log('Move timed out. Check connection and pose before reinitializing.')
            self.initialized=False
        self.refresh_controls()
        self.root.after(20,self.poll)

    def fail(self,message):
        self.log(message); self.plan=[]; self.pending=None; self.busy=False; self.invalidate()
        self.initialized=False; self.stop_pending=False
        if self.ready and self.link:
            try: self.link.send('STOP')
            except Exception: self.ready=False
        self.refresh_controls()

    def stop(self):
        # Cancel future sends first; send STOP before redrawing anything.
        self.plan=[]; self.pending=None; self.busy=False; self.stopped=True
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

    def update_position(self):
        for j,s in self.sliders.items(): s.set(self.positions[j])
        if self.cal:
            q=ik.servo_to_q(self.cal,self.positions)
            xyz=ik.position(self.m,q)*1000
            self.readout.set('Current estimate XYZ mm: '+np.array2string(xyz,precision=1,suppress_small=True)+
                ' | base/shoulder/elbow/wrist deg: '+np.array2string(np.rad2deg(q),precision=1,suppress_small=True))
        else: self.readout.set('Raw S1..S5: '+str(self.positions)+' | calibration needed for XYZ')
        if self.preview and self.positions!=self.preview['start']: self.invalidate()
        if self.preview: self.redraw_preview()
        else: self.draw()

    def calculate(self):
        if self.busy or getattr(self,'stop_pending',False): self.log('Wait for motion/STOP confirmation first.'); return
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
            q,r=ik.inverse_kinematics(target,self.m,q0)
            self.log(f'XYZ {target.tolist()} mm: {r["status"]}; {len(r["history"])-1} updates; residual {r["error"]*1000:.4f} mm')
            if not r['success']:
                self.feedback(f'No move prepared: {r["status"]}; position error {r["error"]*1000:.2f} mm. Try a reachable target or another clear starting pose.','error')
                self.draw(q,target,r); return
            waypoints,miss=ik.plan_motion(self.m,self.cal,start,target,r,MAX_MISS_MM)
            self.preview=dict(start=start,target=target,q=q,result=r,waypoints=waypoints)
            self.move_button.config(state='normal')
            self.feedback(f'Preview ready — IK error {r["error"]*1000:.3f} mm; rounded-command miss {miss:.2f} mm. Move Arm to execute.','success')
            self.log(f'Angles deg {np.rad2deg(q).round(2).tolist()}; rounded miss {miss:.2f} mm; {len(waypoints)} checked J waypoints. Preview only.')
            self.draw(q,target,r)
        except Exception as e:
            message='Cannot prepare move: '+str(e)
            self.log(message); self.feedback(message,'error')
        finally: self.refresh_controls()

    def move(self):
        if self.busy or getattr(self,'stop_pending',False) or self.mode!='I' or not self.initialized or not self.preview: return
        p=self.preview
        try:
            if not np.array_equal(p['target'],[float(e.get()) for e in self.entries]):
                raise ValueError('Target changed; calculate again')
            if self.positions!=p['start']: raise ValueError('Arm moved; calculate again')
            waypoints,_=ik.plan_motion(self.m,self.cal,self.positions,p['target'],p['result'],MAX_MISS_MM)
            self.invalidate(); self.launch(waypoints)
        except Exception as e: self.fail(str(e))

    def manual(self):
        if self.busy or getattr(self,'stop_pending',False) or not self.initialized or self.mode!='A':
            self.log('Manual controls require initialized APP mode and no active move.'); return
        self.invalidate()
        try: self.launch([[int(self.sliders[j].get()) for j in range(5)]])
        except Exception as e: self.fail(str(e))

    def grip(self,closed):
        if self.busy or getattr(self,'stop_pending',False) or not self.initialized or self.mode=='K' or not self.cal: return
        self.invalidate(); pose=list(self.positions)
        pose[0]=self.cal.claw_closed if closed else self.cal.claw_open
        try: self.launch([pose])
        except Exception as e: self.fail(str(e))

    def reload(self):
        if self.synthetic:
            self.log('Synthetic simulation ignores calibration.json. Restart with --simulate --use-calibration to test a saved calibration; hardware files have not been changed.')
            return
        if self.busy or getattr(self,'stop_pending',False): self.log('Wait for movement/STOP confirmation before reloading calibration.'); return
        try:
            cal=mc.Calibration.load(calibration_path()); m=ik.model(GEOMETRY,cal)
            self.cal=cal; self.m=m; self.synthetic=False
            self.invalidate(); self.initialized=False; self.update_init_text()
            for j,s in self.sliders.items():
                lo,hi=self.limit(j); s.config(from_=hi,to=lo)
            if getattr(self.link,'simulated',False):
                self.log('Calibration loaded. Reinitializing emulator upright for IK preview.')
                self.initialize()
            else:
                self.log('Calibration loaded; servos require explicit initialization. Arrange the arm upright before first enable, then Straighten arm upright.')
        except Exception as e: self.log(str(e))
        finally: self.refresh_controls()

    def redraw_preview(self):
        """Retain the same calculated preview when selecting/resizing a view."""
        if self._preview_plot is None: self.draw()
        else: self.draw(*self._preview_plot)

    def draw(self,q=None,target=None,result=None):
        c=self.canvas; c.delete('all')
        candidate=q is not None
        self._preview_plot=(q,target,result) if candidate else None
        current_q=self.m.q0
        if self.cal:
            try:
                current_q=ik.servo_to_q(self.cal,self.positions if self.initialized else self.start_pose())
            except ValueError:
                pass  # No valid upright reference; initialization reports why.
        current=ik.fk(self.m,current_q)[1]*1000
        P=ik.fk(self.m,q)[1]*1000 if candidate else current
        width=c.winfo_width(); height=c.winfo_height()
        if width<=1: width=840
        if height<=1: height=340
        if width<600 or height<180:
            c.create_text(width/2,height/2,text='Enlarge the window to show the 3D preview.',fill='#64748b')
            return
        view=self.preview_view.get()
        axes=np.eye(3)*70.
        fit_points=np.vstack((current,P,[[0,0,0]],axes))
        if target is not None: fit_points=np.vstack((fit_points,target))
        xy=fit_points[:,:2]
        low=xy.min(axis=0)-12; high=xy.max(axis=0)+12
        floor=np.array([[low[0],low[1],0],[high[0],low[1],0],
                        [high[0],high[1],0],[low[0],high[1],0]])
        project=fit_preview_projection(np.vstack((fit_points,floor)),view,
                                       (28,72,width*.50-40,height-62))
        c.create_polygon(*[v for p in floor for v in project(p)],
                         fill='#f1f5f9',outline='#cbd5e1',tags='table_plane')
        for axis in (0,1):
            for value in np.arange(math.ceil(low[axis]/50)*50,high[axis],50):
                a=np.r_[low,0.]; b=np.r_[high,0.]; a[axis]=b[axis]=value
                c.create_line(*project(a),*project(b),fill='#dce4ed')
        def arm(points,color,dashed=False):
            tag='current_link' if dashed else 'preview_link'
            for a,b in zip(points[:-1],points[1:]):
                if np.linalg.norm(b-a)>1e-9:
                    c.create_line(*project(a),*project(b),fill=color,width=3 if dashed else 4,
                                  dash=(5,3) if dashed else (),tags=tag)
            for p in points:
                x,y=project(p);c.create_oval(x-3,y-3,x+3,y+3,fill=color,outline=color)
        if candidate: arm(current,'#94a3b8',True)
        arm(P,'#7048a5')
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
        c.create_text(width*.25,14,text='World frame {0}: X right, Y forward, Z up')
        c.create_text(width*.25,36,text=view,fill='#475569')
        legend='Gray: current estimate | Purple: calculated | Red: target' if candidate else (
            'Purple: current estimated pose' if self.initialized else 'Purple: startup model pose (not initialized)')
        c.create_text(width*.25,height-18,text=legend,fill='#475569')
        c.create_text(width*.75,14,text='Position error (mm), logarithmic scale')
        if result is not None:
            errors=np.maximum(result['errors']*1000,1e-6); logs=np.log10(errors)
            lo=math.floor(min(-1.,float(logs.min()))); hi=math.ceil(max(0.,float(logs.max())))
            left=width*.57; right=width-20; top=68; bottom=height-58
            for power in range(lo,hi+1,max(1,math.ceil((hi-lo)/6))):
                y=bottom-(bottom-top)*(power-lo)/(hi-lo)
                c.create_line(left,y,right,y,fill='#e2e8f0')
                c.create_text(left-6,y,text=f'{10.**power:g}',anchor='e',fill='#64748b')
            coords=[]
            for i,v in enumerate(logs):coords.extend((left+(right-left)*i/max(1,len(logs)-1),bottom-(bottom-top)*(v-lo)/(hi-lo)))
            if len(coords)>2: c.create_line(*coords,fill='#1971c2',width=2)
            c.create_text(left,bottom+15,text='0',fill='#64748b')
            c.create_text(right,bottom+15,text=str(len(errors)-1),fill='#64748b')
            c.create_text(width*.75,height-18,text=f'{result["status"]}: {len(errors)-1} updates; final {errors[-1]:.4f} mm')
        else:
            c.create_text(width*.75,height*.5,text='Calculate / Preview to show convergence',fill='#64748b')

    def save_log(self):
        from tkinter import filedialog
        path=filedialog.asksaveasfilename(defaultextension='.txt',initialfile='Lab03_experiment_log.txt')
        if path:
            with open(path,'w',encoding='utf-8') as f: f.write(self.terminal.get('1.0','end'))
            self.log('Log saved: '+path)

    def close(self):
        self.stop()
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

