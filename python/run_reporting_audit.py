"""Archive actual executions of the existing Fang/Zhang reproduction solvers.
No plant, control gain, delay, or history convention is silently corrected.
Run: python python/run_reporting_audit.py
"""
from __future__ import annotations
import contextlib, hashlib, io, json, platform, subprocess, sys, time, traceback
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import scipy
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python/src'))
from inexact_predictor import run_inexact_predictor
from uncompensated import run_uncompensated
from delay_free import run_delay_free

def main():
    out=ROOT/'results/reporting-audit'; out.mkdir(parents=True,exist_ok=True)
    rows=[]
    cases=[('predictor',run_inexact_predictor,{'dt':d}) for d in (.002,.001,.0005)]
    cases += [('uncompensated',run_uncompensated,{'dt':.001}),
              ('delay-free',run_delay_free,{'dt':.001})]
    cases += [('horizon-sweep',run_inexact_predictor,{'D0':d,'dt':.002}) for d in (.4,.42,.48,.5)]
    for i,(name,fn,params) in enumerate(cases):
        row={'case':name,'function':fn.__name__,'overrides':params,'status':'pending'}
        start=time.perf_counter(); log=io.StringIO()
        try:
            with contextlib.redirect_stdout(log): result=fn(params)
            t=np.asarray(result['t']); x=np.asarray(result['x_hist'])
            u=np.column_stack((result['u1_hist'],result['u2_hist']))
            finite=bool(np.isfinite(np.column_stack((x,u))).all())
            row.update(status='completed' if finite else 'nonfinite',finite=finite,
                       t_end_actual=float(t[-1]),sample_count=len(t),initial_X=x[0].tolist(),
                       final_X=x[-1].tolist(),final_X_norm=float(np.linalg.norm(x[-1])),
                       initial_U=u[0].tolist(),max_abs_U=np.nanmax(np.abs(u),axis=0).tolist(),
                       max_X_norm=float(np.nanmax(np.linalg.norm(x,axis=1))),
                       early_termination=bool(t[-1]<params.get('t_end',10.)-1e-9))
            path=f'{i:02d}-{name}.csv'
            np.savetxt(out/path,np.column_stack((t,x,u)),delimiter=',',comments='',header='t,X1,X2,U1,U2')
            row['trajectory']=path
        except Exception: row.update(status='exception',traceback=traceback.format_exc())
        row['elapsed_seconds']=time.perf_counter()-start
        (out/f'{i:02d}-{name}.log').write_text(log.getvalue(),encoding='utf-8')
        rows.append(row)
        (out/'runs.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(row),flush=True)
    def git(*args):
        p=subprocess.run(['git','-C',str(ROOT),*args],capture_output=True,text=True)
        return p.stdout.strip() if p.returncode==0 else None
    metadata={'created_utc':datetime.now(timezone.utc).isoformat(),'tested_checkout':git('rev-parse','HEAD'),
              'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,
              'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted((ROOT/'python/src').glob('*.py'))},
              'scope':'Existing solvers, original Eqs. (35)-(36), existing gains and histories; no solver edits.',
              'limitations':['Forward Euler and nearest-grid input lookup are retained.',
                             'All specified cases retained; finite runs do not prove exponential stability.',
                             'No pixel-based match to the published figures.']}
    (out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    if any(r['status']!='completed' for r in rows): raise SystemExit(1)

if __name__=='__main__': main()
