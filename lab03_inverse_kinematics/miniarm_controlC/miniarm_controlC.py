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
JOINT_NAMES = ["Claw", "Wrist", "Forearm", "Arm", "Base"]
DISPLAY_ORDER = [BASE, ARM, FOREARM, WRIST, CLAW]   # left to right on screen: bottom of the arm first

MODES = {   # code sent to the Arduino: (button text, color)
    "A": ("APP CONTROL", "#2b8a3e"),
    "K": ("KNOB CONTROL", "#e8590c"),
    "I": ("IK CONTROL", "#1c7ed6"),
}


def calibration_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), CALIBRATION_FILE)


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
        inbox.put("READY")

    def send(self, line):
        parts = line.split()
        cmd, nums = parts[0], [int(p) for p in parts[1:] if p.lstrip("-").isdigit()]
        clamp = lambda i, v: max(self.limits[i][0], min(self.limits[i][1], v))
        if cmd == "CFG" and len(nums) == 12:
            self.limits = [(nums[2 * i], nums[2 * i + 1]) for i in range(5)]
            self.inbox.put("CFG OK")
        elif cmd == "INIT" and len(nums) == 5:
            self.cur = [clamp(i, v) for i, v in enumerate(nums)]
            self.tgt = list(self.cur)
            self.on = self.changed = True
            self.inbox.put("OK")
        elif cmd == "MODE":
            self.tgt, self.move_active, self.seq = list(self.cur), False, None
            self.mode = parts[1]
            self.inbox.put(f"MODE {self.mode}")
        elif cmd in ("M", "J") and len(nums) == 5:
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
        while self.on and now - self.last_step >= STEP_MS / 1000:
            self.last_step = now if now - self.last_step > 1 else self.last_step + STEP_MS / 1000
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
                self.seq = [targets, idx, now + SETTLE_MS / 1000 if arrived else 0.0]
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
    def __init__(self, root, simulate=False, port='COM11'):
        self.root=root; root.title('miniArm 3D XYZ Control')
        self.inbox=queue.Queue(); self.link=None; self.ready=False
        self.mode='A'; self.positions=[90]*5; self.commanded=[90]*5
        self.initialized=False; self.busy=False; self.pending=None
        self.preview=None; self.plan=[]; self.active=None; self.started=0
        self.knobs=None; self.knob_baseline=None; self.synthetic=False
        try: self.cal=mc.Calibration.load(calibration_path())
        except mc.CalibrationError: self.cal=None
        if simulate and self.cal is None:
            self.cal=ik.demo_calibration(); self.synthetic=True
        self.m=ik.model(GEOMETRY,self.cal)
        self._build()
        if simulate:
            self.link=SimulatedArm(self.inbox)
        elif serial is None:
            self.log('pyserial missing. Install requirements or use --simulate.')
        else:
            try: self.link=SerialLink(port,BAUD_RATE,self.inbox)
            except Exception as e: self.log(f'Connection failed: {e}. Use --simulate for offline work.')
        self.status.set('SIMULATION: no physical movement' if simulate else f'Hardware port: {port}')
        if self.synthetic: self.log('SIMULATION uses a synthetic calibration, not measurements.')
        self.log('Connect leaves servos off. Place the arm at the displayed startup pose, then Initialize.')
        self.root.after(20,self.poll)
        root.protocol('WM_DELETE_WINDOW',self.close)

    def _build(self):
        bar=tk.Frame(self.root); bar.pack(fill='x',padx=10,pady=8)
        for mode in ('A','K','I'):
            tk.Button(bar,text=MODES[mode][0],command=lambda m=mode:self.set_mode(m)).pack(side='left',padx=3)
        tk.Button(bar,text='STOP',bg='#c92a2a',fg='white',command=self.stop).pack(side='left',padx=6)
        tk.Button(bar,text='Reload calibration',command=self.reload).pack(side='left')
        self.status=tk.StringVar(); tk.Label(self.root,textvariable=self.status).pack()
        setup=tk.Frame(self.root); setup.pack(fill='x',padx=12)
        self.init_text=tk.StringVar(); tk.Label(setup,textvariable=self.init_text).pack(side='left')
        tk.Button(setup,text='Initialize at startup pose',command=self.initialize).pack(side='right')
        self.update_init_text()
        panel=tk.Frame(self.root); panel.pack(padx=10)
        self.sliders={}
        for col,j in enumerate(DISPLAY_ORDER):
            box=tk.Frame(panel); box.grid(row=0,column=col,padx=7)
            lo,hi=self.limit(j)
            s=tk.Scale(box,from_=hi,to=lo,length=140,label=JOINT_NAMES[j])
            s.set(90); s.pack(); self.sliders[j]=s
        tk.Button(panel,text='Send manual sliders (APP mode)',command=self.manual).grid(row=1,column=0,columnspan=5)
        inp=tk.Frame(self.root); inp.pack(pady=8)
        self.entries=[]
        for col,(name,value) in enumerate(zip(('X right (mm)','Y forward (mm)','Z up (mm)'),(-40,100,150))):
            tk.Label(inp,text=name).grid(row=0,column=col,padx=10)
            e=tk.Entry(inp,width=12); e.insert(0,str(value)); e.grid(row=1,column=col,padx=10)
            e.bind('<KeyRelease>',lambda event:self.invalidate()); self.entries.append(e)
        actions=tk.Frame(self.root); actions.pack()
        tk.Button(actions,text='Calculate / Preview',command=self.calculate).pack(side='left',padx=4)
        self.move_button=tk.Button(actions,text='Move Arm',state='disabled',command=self.move)
        self.move_button.pack(side='left',padx=4)
        tk.Button(actions,text='Open claw',command=lambda:self.grip(False)).pack(side='left',padx=4)
        tk.Button(actions,text='Close claw',command=lambda:self.grip(True)).pack(side='left',padx=4)
        tk.Button(actions,text='Save experiment log',command=self.save_log).pack(side='left',padx=4)
        self.readout=tk.StringVar(value='Position estimate: not initialized')
        tk.Label(self.root,textvariable=self.readout,font=('Consolas',9)).pack(pady=5)
        self.canvas=tk.Canvas(self.root,width=750,height=250,bg='white'); self.canvas.pack(padx=10)
        self.terminal=scrolledtext.ScrolledText(self.root,width=100,height=9,state='disabled',font=('Consolas',9))
        self.terminal.pack(padx=10,pady=8)
        self.draw()

    def log(self,text):
        self.terminal.config(state='normal'); self.terminal.insert('end',text+'\n')
        self.terminal.see('end'); self.terminal.config(state='disabled')

    def limit(self,j):
        return self.cal.limits(j) if self.cal else (0,180)

    def update_init_text(self):
        pose=self.start_pose()
        self.init_text.set('Startup S1..S5 raw commands: '+' '.join(map(str,pose)))

    def start_pose(self):
        if self.cal:
            try: return ik.q_to_servo(self.cal,self.m.q0,self.cal.claw_open)
            except ValueError: return self.cal.middle_pose()
        return [90]*5

    def send(self,line):
        if not self.ready or not self.link: raise ValueError('Arduino is not ready')
        self.link.send(line)

    def invalidate(self):
        self.preview=None; self.move_button.config(state='disabled')

    def set_mode(self,mode):
        if self.busy:
            self.log('Press STOP before changing mode.'); return
        self.invalidate(); self.mode=mode
        self.knob_baseline=list(self.knobs) if self.knobs else None
        if self.ready:
            try: self.send('MODE '+mode)
            except Exception as e: self.log(str(e))
        self.log('Mode: '+MODES[mode][0])

    def initialize(self):
        if not self.ready or self.busy: return
        try:
            self.invalidate()
            limits=' '.join(f'{lo} {hi}' for lo,hi in (self.limit(j) for j in range(5)))
            self.send(f'CFG {limits} {STEP_MS} {SETTLE_MS}')
            self.mode='A'; self.send('MODE A')
            self.launch([self.start_pose()],command='INIT')
        except Exception as e: self.log(str(e))

    def launch(self,poses,command='J'):
        self.plan=[list(p) for p in poses]; self.active=command
        self.busy=True; self.next_waypoint()

    def next_waypoint(self):
        if not self.plan:
            self.busy=False; self.pending=None
            self.log('Command sequence finished (estimated position; no encoder feedback).'); return
        self.pending=self.plan.pop(0); self.started=time.monotonic()
        self.send(self.active+' '+' '.join(map(str,self.pending)))

    def poll(self):
        if self.link: self.link.tick()
        while not self.inbox.empty():
            line=self.inbox.get(); parts=line.split()
            if not parts: continue
            if line.endswith('READY'):
                self.ready=True; self.initialized=False; self.busy=False
                self.plan=[]; self.pending=None; self.invalidate()
                self.log('Arduino READY. Servos off; initialize explicitly.')
            elif parts[0]=='P' and len(parts)==6:
                self.positions=[int(v) for v in parts[1:]]; self.update_position()
            elif line=='OK' and self.pending is not None:
                self.positions=list(self.pending); self.commanded=list(self.pending)
                if self.active=='INIT': self.initialized=True
                self.pending=None; self.update_position()
                try: self.next_waypoint()
                except Exception as e: self.fail(str(e))
            elif parts[0]=='K' and len(parts)==6:
                self.knobs=[int(v) for v in parts[1:]]
                if self.knob_baseline is None: self.knob_baseline=list(self.knobs)
                threshold=KNOB_TAKEOVER_DEG/180*1023
                if self.initialized and self.mode!='K' and any(abs(a-b)>threshold for a,b in zip(self.knobs,self.knob_baseline)):
                    self.stop(); self.set_mode('K'); self.log('Knob takeover: IK preview cancelled.')
            elif parts[0]=='ERR': self.fail(line)
        if self.busy and time.monotonic()-self.started>30:
            self.stop(); self.log('Move timed out. Check connection and pose before reinitializing.')
            self.initialized=False
        self.root.after(20,self.poll)

    def fail(self,message):
        self.log(message); self.plan=[]; self.pending=None; self.busy=False; self.invalidate()
        self.initialized=False
        if self.ready and self.link:
            try: self.link.send('STOP')
            except Exception: self.ready=False

    def stop(self):
        self.plan=[]; self.pending=None; self.busy=False; self.invalidate()
        if self.ready:
            try:
                if self.mode=='K': self.link.send('MODE A'); self.mode='A'
                self.link.send('STOP')
            except Exception as e: self.fail(str(e))
        self.log('STOP requested; queued waypoints cancelled.')

    def update_position(self):
        for j,s in self.sliders.items(): s.set(self.positions[j])
        if self.cal:
            q=ik.servo_to_q(self.cal,self.positions)
            xyz=ik.position(self.m,q)*1000
            self.readout.set('Estimated XYZ mm: '+np.array2string(xyz,precision=1)+
                ' | base/shoulder/elbow/wrist deg: '+np.array2string(np.rad2deg(q),precision=1))
        else: self.readout.set('Raw S1..S5: '+str(self.positions)+' | calibration needed for XYZ')
        if self.preview and self.positions!=self.preview['start']: self.invalidate()
        self.draw()

    def calculate(self):
        if self.busy: self.log('Wait for motion or press STOP first.'); return
        self.invalidate()
        if not self.cal or not self.initialized:
            self.log('Load calibration and Initialize before calculating.'); return
        if self.mode!='I': self.log('Select IK CONTROL first.'); return
        try:
            target=np.array([float(e.get()) for e in self.entries])
            if not np.isfinite(target).all(): raise ValueError('XYZ must be finite numbers')
            start=list(self.positions); q0=ik.servo_to_q(self.cal,start)
            q,r=ik.inverse_kinematics(target,self.m,q0)
            self.log(f'XYZ {target.tolist()} mm: {r["status"]}; {len(r["history"])-1} updates; residual {r["error"]*1000:.4f} mm')
            if not r['success']:
                self.draw(q,target,r); return
            waypoints,miss=ik.plan_motion(self.m,self.cal,start,target,r,MAX_MISS_MM)
            self.preview=dict(start=start,target=target,q=q,result=r,waypoints=waypoints)
            self.move_button.config(state='normal')
            self.log(f'Angles deg {np.rad2deg(q).round(2).tolist()}; rounded miss {miss:.2f} mm; {len(waypoints)} checked J waypoints. Preview only.')
            self.draw(q,target,r)
        except Exception as e: self.log('Cannot prepare move: '+str(e))

    def move(self):
        if self.busy or self.mode!='I' or not self.initialized or not self.preview: return
        p=self.preview
        try:
            if not np.array_equal(p['target'],[float(e.get()) for e in self.entries]):
                raise ValueError('Target changed; calculate again')
            if self.positions!=p['start']: raise ValueError('Arm moved; calculate again')
            waypoints,_=ik.plan_motion(self.m,self.cal,self.positions,p['target'],p['result'],MAX_MISS_MM)
            self.invalidate(); self.launch(waypoints)
        except Exception as e: self.fail(str(e))

    def manual(self):
        if self.busy or not self.initialized or self.mode!='A':
            self.log('Manual controls require initialized APP mode and no active move.'); return
        self.invalidate()
        try: self.launch([[int(self.sliders[j].get()) for j in range(5)]])
        except Exception as e: self.fail(str(e))

    def grip(self,closed):
        if self.busy or not self.initialized or self.mode=='K' or not self.cal: return
        self.invalidate(); pose=list(self.positions)
        pose[0]=self.cal.claw_closed if closed else self.cal.claw_open
        try: self.launch([pose])
        except Exception as e: self.fail(str(e))

    def reload(self):
        if self.busy: self.log('Stop movement before reloading calibration.'); return
        try:
            cal=mc.Calibration.load(calibration_path()); m=ik.model(GEOMETRY,cal)
            self.cal=cal; self.m=m; self.synthetic=False
            self.invalidate(); self.initialized=False; self.update_init_text()
            for j,s in self.sliders.items():
                lo,hi=self.limit(j); s.config(from_=hi,to=lo)
            self.log('Calibration loaded. Initialize again at the displayed pose.')
        except Exception as e: self.log(str(e))

    def draw(self,q=None,target=None,result=None):
        c=self.canvas; c.delete('all')
        # Fixed isometric camera; all coordinates are displayed in millimetres.
        if q is None:
            q=ik.servo_to_q(self.cal,self.positions) if self.cal and self.initialized else self.m.q0
        P=ik.fk(self.m,q)[1]*1000
        project=lambda p:(230+.65*(p[0]-p[1]),220-.60*p[2]+.20*(p[0]+p[1]))
        for end,label,color in (([100,0,0],'X right','#e03131'),([0,100,0],'Y forward','#2b8a3e'),([0,0,100],'Z up','#1971c2')):
            a=project([0,0,0]); b=project(end); c.create_line(*a,*b,fill=color,arrow='last'); c.create_text(b[0],b[1]-9,text=label,fill=color)
        for a,b in zip(P[:-1],P[1:]): c.create_line(*project(a),*project(b),fill='#7048a5',width=4)
        for p in P:
            x,y=project(p); c.create_oval(x-4,y-4,x+4,y+4,fill='#7048a5')
        if target is not None:
            x,y=project(target); c.create_line(x-6,y-6,x+6,y+6,fill='red',width=2); c.create_line(x-6,y+6,x+6,y-6,fill='red',width=2)
        c.create_text(200,12,text='3D isometric preview (purple); target (red)')
        c.create_text(560,12,text='Position error (mm), logarithmic scale')
        if result is not None:
            errors=np.maximum(result['errors']*1000,1e-6); logs=np.log10(errors)
            lo=min(-1.,float(logs.min())); hi=max(0.,float(logs.max()))
            coords=[]
            for i,v in enumerate(logs): coords.extend((435+290*i/max(1,len(logs)-1),220-180*(v-lo)/max(1e-6,hi-lo)))
            if len(coords)>2: c.create_line(*coords,fill='#1971c2',width=2)
            c.create_text(580,240,text=f'Solver updates: {len(errors)-1}; final {errors[-1]:.4f} mm')

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
    p.add_argument('--simulate',action='store_true')
    p.add_argument('--port',default=SERIAL_PORT)
    args=p.parse_args()
    root=tk.Tk(); app=ArmApp(root,simulate=args.simulate,port=args.port); root.mainloop()
