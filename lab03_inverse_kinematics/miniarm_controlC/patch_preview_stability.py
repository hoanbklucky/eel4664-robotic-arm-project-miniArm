"""Patch an existing/custom miniArm controller without replacing its UI.

Run: python patch_preview_stability.py miniarm_controlC.py
Creates an exact byte backup before editing. Does not connect to the robot.
"""
import ast
from pathlib import Path
import sys

MARKER='# Stable preview behavior patch 2.0.2'
HELPER="def preview_workspace_points(m,target=None):\n    \"\"\"Fixed workspace scale; independent of live and unsent servo values.\"\"\"\n    shoulder=ik.fk(m,np.zeros(len(m.mdh)))[1][1]*1000\n    reach=max(70.,float(1000*(np.sum(np.abs(m.mdh[1:,0]))+\n                  np.sum(np.abs(m.mdh[1:,2]))+np.linalg.norm(m.tool[:3,3]))))\n    points=[shoulder+reach*np.array([x,y,z])\n            for x in (-1,1) for y in (-1,1) for z in (-1,1)]\n    points.extend([np.zeros(3),*list(np.eye(3)*70.)])\n    if target is not None:points.append(np.asarray(target,dtype=float))\n    return np.asarray(points)\n\n\n"
INVALIDATE="    def invalidate(self,clear_display=False):\n        \"\"\"Invalidate motion without erasing the last calculated drawing.\"\"\"\n        self.preview=None\n        self.move_button.config(state='disabled')\n        if clear_display:self._preview_plot=None\n        retained=getattr(self,'_preview_plot',None) is not None\n        message='Last calculated preview retained — calculate again before Move Arm.' if retained else 'Calculate / Preview prepares a move; editing inputs does not move the arm.'\n        self.feedback(message if self.mode=='I' else '')\n        if hasattr(self,'canvas'):self.redraw_preview()\n        self.refresh_controls()\n"
def patch_source(source):
    source=source.replace('\r\n','\n')
    if MARKER in source:return source
    tree=ast.parse(source)
    arm=next((n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ArmApp'),None)
    if arm is None:raise ValueError('Expected class ArmApp; no file was changed')
    methods={n.name:n for n in arm.body if isinstance(n,ast.FunctionDef)}
    required=('invalidate','draw','update_position','poll','reload')
    if any(name not in methods for name in required):
        raise ValueError('Controller structure differs; no file was changed')
    lines=source.splitlines(keepends=True)
    edits=[]
    for name in required:
        node=methods[name];text=''.join(lines[node.lineno-1:node.end_lineno])
        if name=='invalidate':text=INVALIDATE
        elif name=='draw':
            old="        fit_points=np.vstack((current,P,[[0,0,0]],axes))\n        if target is not None: fit_points=np.vstack((fit_points,target))"
            if old not in text and 'fit_points=preview_workspace_points(self.m,target)' not in text:
                raise ValueError('Drawing bounds differ; no file was changed')
            text=text.replace(old,'        fit_points=preview_workspace_points(self.m,target)')
            text=text.replace('        if unsent is not None: fit_points=np.vstack((fit_points,unsent))\n','')
        elif name=='update_position':
            text=text.replace('        if self.preview: self.redraw_preview()\n        else: self.draw()',
                              '        self.redraw_preview()  # retain the last calculated pose')
        elif name in ('poll','reload'):
            text=text.replace('self.invalidate()','self.invalidate(clear_display=True)')
        edits.append((node.lineno-1,node.end_lineno,text.rstrip('\n')+'\n'))
    for start,end,text in sorted(edits,reverse=True):
        lines[start:end]=[text]
    updated=''.join(lines)
    if not any(isinstance(n,ast.FunctionDef) and n.name=='preview_workspace_points' for n in tree.body):
        updated=HELPER+updated
    # Keep customized APP slider feedback working alongside the retained IK pose.
    updated=updated.replace("if not candidate and self.cal and getattr(self,'mode','A')=='A' and self.initialized:",
                            "if self.cal and getattr(self,'mode','A')=='A' and self.initialized:")
    updated=updated.replace("if self.mode=='A' and hasattr(self,'canvas') and self._preview_plot is None: self.draw()",
                            "if self.mode=='A' and hasattr(self,'canvas'): self.redraw_preview()")
    updated=updated.replace("if hasattr(self,'canvas') and getattr(self,'_preview_plot',None) is None: self.draw()",
                            "if hasattr(self,'canvas'): self.redraw_preview()")
    updated=updated.replace('    firmware_version = APP_VERSION',"    firmware_version = '2.0.0'")
    # Stamp the controller only; do not change firmware, calibration or modules.
    parsed=ast.parse(updated)
    imported=next((n for n in parsed.body if isinstance(n,ast.ImportFrom) and n.module=='miniarm_version'),None)
    if imported is not None:
        parts=updated.splitlines(keepends=True)
        parts.insert(imported.end_lineno,"APP_VERSION = '2.0.2'  # stable-preview controller patch\n")
        updated=''.join(parts)
    updated=MARKER+'\n'+updated
    ast.parse(updated)  # refuse to write syntactically invalid Python
    return updated

def apply(path):
    path=Path(path)
    raw=path.read_bytes();source=raw.decode('utf-8-sig')
    patched=patch_source(source)
    if MARKER in source:
        print('Preview fix already applied; no files changed.');return
    backup=path.with_name(path.name+'.before_preview_fix')
    if backup.exists():raise ValueError(f'Backup already exists: {backup}; preserve it before retrying')
    backup.write_bytes(raw)
    path.write_text(patched,encoding='utf-8',newline='\n')
    print(f'Updated {path}; original backup: {backup}')
    print('Restart the GUI. No firmware upload or recalibration is needed.')

if __name__=='__main__':
    if len(sys.argv)!=2:raise SystemExit('Usage: python patch_preview_stability.py miniarm_controlC.py')
    try:apply(sys.argv[1])
    except (OSError,UnicodeError,ValueError,SyntaxError) as error:raise SystemExit(str(error))
