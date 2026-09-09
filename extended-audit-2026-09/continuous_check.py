"""Independent continuous-feedback discretization and local spectrum audit.

Physical DDE: RK4 using piecewise-linear *past* commands. At each node the
Volterra predictor is recomputed and the current command is solved by fixed
point iteration. No extrapolated future physical state or target generates x.
Initial command jump is NOT interpolated into negative-time zero history.
Interpolation is a method-of-steps approximation, not an online FOH controller.
"""
from __future__ import annotations
import json, argparse, hashlib, platform, time
from pathlib import Path
import numpy as np
from numba import njit

@njit(cache=True)
def f(a,b,u,v,linear):
    if linear: return b,b+2*u+v
    return b,-a+np.sin(a)+b+u+np.tanh(u)+np.sin(v)

@njit(cache=True)
def ramp(a,b,h,u0,v0,u1,v1,linear):
    k1,l1=f(a,b,u0,v0,linear)
    k2,l2=f(a+h*k1/2,b+h*l1/2,(u0+u1)/2,(v0+v1)/2,linear)
    k3,l3=f(a+h*k2/2,b+h*l2/2,(u0+u1)/2,(v0+v1)/2,linear)
    k4,l4=f(a+h*k3,b+h*l3,u1,v1,linear)
    return a+h*(k1+2*k2+2*k3+k4)/6,b+h*(l1+2*l2+2*l3+l4)/6

@njit(cache=True)
def predict(a,b,U,k,dt,N0,uend,vend,linear):
    for j in range(k-N0,k):
        if j<0: u0=v0=u1=v1=0.
        else:
            u0,v0=U[j,0],U[j,1]
            if j+1==k: u1,v1=uend,vend
            else: u1,v1=U[j+1,0],U[j+1,1]
        a,b=ramp(a,b,dt,u0,v0,u1,v1,linear)
    return a,b

@njit(cache=True)
def simulate(dt=.001,T=10.,D0=.45,D1=.4,D2=.5,x0=1.,linear=False):
    N0=int(round(D0/dt));N1=int(round(D1/dt));N2=int(round(D2/dt));n=int(round(T/dt))
    if min(N0,N1,N2)<1 or max(abs(N0*dt-D0),abs(N1*dt-D1),abs(N2*dt-D2),abs(n*dt-T))>1e-10:
        raise ValueError('This cross-check requires commensurate positive delays and horizon')
    X=np.zeros((n+1,2));U=np.zeros_like(X);P=np.zeros_like(X);X[0,:]=x0
    residual=0.;maxit=0
    for k in range(n+1):
        u,v=(0.,0.) if k==0 else (U[k-1,0],U[k-1,1])
        for it in range(40):
            p,q=predict(X[k,0],X[k,1],U,k,dt,N0,u,v,linear)
            un,vn=-3*p-5*q,-10*p-10*q
            err=max(abs(un-u),abs(vn-v))
            u,v=un,vn
            if err<2e-13*(1+max(abs(u),abs(v))): break
        else: raise RuntimeError('Predictor endpoint fixed point did not converge')
        maxit=max(maxit,it+1);residual=max(residual,err)
        P[k,0]=p;P[k,1]=q;U[k,0]=u;U[k,1]=v
        if not np.isfinite(p+q+u+v) or abs(X[k,0])+abs(X[k,1])>1e9:
            raise FloatingPointError('Explicit safety limit/nonfinite')
        if k==n:break
        j=k-N1;ell=k-N2
        # A bin ending at time zero uses left history, preserving the startup jump.
        u0,u1=(0.,0.) if j<0 else (U[j,0],U[j+1,0])
        v0,v1=(0.,0.) if ell<0 else (U[ell,1],U[ell+1,1])
        X[k+1,0],X[k+1,1]=ramp(X[k,0],X[k,1],dt,u0,v0,u1,v1,linear)
    return np.arange(n+1)*dt,X,U,P,residual,maxit

def spectral_audit():
    """High precision roots plus original integral/physical eigenmode residuals.

Numerical roots are evidence, not a validated interval certificate of the full
spectrum. One positive-real-part root suffices for a local instability finding.
"""
    import mpmath as mp
    mp.mp.dps=70
    A=mp.matrix([[0,1],[0,1]]);C1=mp.matrix([[0,0],[-6,-10]]);C2=mp.matrix([[0,0],[-10,-10]])
    I=mp.eye(2);D0=mp.mpf('.45');D1=mp.mpf('.4');D2=mp.mpf('.5');E=mp.expm(A*D0)
    def M(s):
        return s*I-A-C1-C2-E*(C1*(mp.exp(-s*D1)-mp.exp(-s*D0))+C2*(mp.exp(-s*D2)-mp.exp(-s*D0)))
    roots=[]
    for guess in [mp.mpc('.68','53.15'),mp.mpc('.60','39.45'),mp.mpc('-.014','25.99')]:
        s=mp.findroot(lambda s:mp.det(M(s)),(guess,guess+mp.mpc('.01','.02')),tol=mp.mpf('1e-60'))
        C=s*I-A
        Q=C**-1*(I-mp.expm(-C*D0))*(C1+C2)
        p=mp.matrix([1,-M(s)[0,0]/M(s)[0,1]])
        x=E**-1*(I-Q)*p
        res=(s*I-A)*x-(C1*mp.exp(-s*D1)+C2*mp.exp(-s*D2))*p
        roots.append(dict(real=mp.nstr(s.real,40),imag=mp.nstr(s.imag,40),
            determinant_residual=mp.nstr(abs(mp.det(M(s))),8),
            original_plant_and_predictor_eigenmode_residual=mp.nstr(max(abs(v) for v in res),8)))
    return dict(A=[[0,1],[0,1]],B1K1=[[0,0],[-6,-10]],B2K2=[[0,0],[-10,-10]],
        formula='det(sI-A-C1-C2-exp(A*D0)*(C1*(exp(-sD1)-exp(-sD0))+C2*(exp(-sD2)-exp(-sD0))))=0',
        D0=.45,D1=.4,D2=.5,mpmath_decimal_digits=70,roots=roots,
        scope='Local linearization at the origin, printed Section 6 parameters; not a refutation of the conditional small-mismatch theorem; not a complete spectrum search.')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('continuous_results'));args=ap.parse_args();out=args.out;out.mkdir(exist_ok=True,parents=True)
    cases=[('continuous_h'+str(h),dict(dt=h)) for h in [.004,.002,.001,.0005]]
    cases[0]=('continuous_h0.005',dict(dt=.005))
    cases += [('small_ic_h0.001',dict(dt=.001,T=35.,x0=1e-6)),
              ('small_ic_h0.0005',dict(dt=.0005,T=25.,x0=1e-6)),
              ('linear_small_ic',dict(dt=.001,T=15.,x0=1e-6,linear=True))]
    rows=[];runs={}
    for name,p in cases:
        record=out/(name+'.json')
        trajectory=out/(name+'.csv')
        if record.exists() and trajectory.exists():
            row=json.loads(record.read_text())
            if row['parameters'] != p: raise ValueError('Resume parameters differ: '+name)
            a=np.loadtxt(trajectory,delimiter=',',skiprows=1)
            runs[name]=(a[:,0],a[:,1:3],a[:,3:5],a[:,5:7],row['fixed_point_max_abs_last_update'],row['max_fixed_point_iterations'])
            rows.append(row);print('RESUMED '+name,flush=True);continue
        start=time.perf_counter();r=simulate(**p);t,x,u,P,err,it=r;runs[name]=r
        row=dict(case=name,parameters=p,completed=True,t_end=float(t[-1]),final_norm=float(np.linalg.norm(x[-1])),
                 initial_U=u[0].tolist(),fixed_point_max_abs_last_update=err,max_fixed_point_iterations=it,
                 interval_peak_state_norms=[dict(t_start=j,t_end=min(j+5,t[-1]),peak=float(np.linalg.norm(x[(t>=j)&(t<=min(j+5,t[-1]))],axis=1).max())) for j in np.arange(0,t[-1],5)],
                 wall_seconds=time.perf_counter()-start)
        rows.append(row);print(json.dumps(row),flush=True)
        np.savetxt(out/(name+'.csv'),np.column_stack([t,x,u,P]),delimiter=',',header='t,x1,x2,u1,u2,P1,P2',comments='')
        record.write_text(json.dumps(row,indent=2)+'\n')
    ref=runs['continuous_h0.0005'];differences=[]
    for h in [.005,.002,.001]:
        t,x,*_=runs['continuous_h'+str(h)]
        xi=np.column_stack([np.interp(t,ref[0],ref[1][:,j]) for j in [0,1]])
        differences.append(dict(dt=h,max_componentwise_difference_to_h00005=float(np.max(abs(x-xi)))))
    (out/'summary.json').write_text(json.dumps(rows,indent=2)+'\n');(out/'refinement.json').write_text(json.dumps(differences,indent=2)+'\n')
    (out/'spectrum.json').write_text(json.dumps(spectral_audit(),indent=2)+'\n')
    (out/'provenance.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),python=platform.python_version(),numpy=np.__version__,execution='Local container, actually executed',method=__doc__),indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for kind in ['state','input']:
        fig,ax=plt.subplots(figsize=(8,4.5));t,x,u,*_=ref;y=x if kind=='state' else u
        ax.plot(t,y[:,0],label=kind+' 1');ax.plot(t,y[:,1],'--',label=kind+' 2')
        ax.set(xlabel='Time (s)',ylabel=kind,title='Continuous-feedback cross-check, h=0.0005');ax.legend();ax.grid(True,alpha=.25)
        fig.tight_layout();fig.savefig(out/(kind+'.png'),dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4.5))
    for name in ['small_ic_h0.001','small_ic_h0.0005','linear_small_ic']:
        t,x,*_=runs[name];ax.semilogy(t,np.maximum(np.linalg.norm(x,axis=1),1e-16),label=name)
    ax.set(xlabel='Time (s)',ylabel='Physical-state norm',title='Small initial state [1e-6,1e-6], zero input prehistory')
    ax.legend();ax.grid(True,alpha=.25);fig.tight_layout();fig.savefig(out/'small_initial_condition.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
