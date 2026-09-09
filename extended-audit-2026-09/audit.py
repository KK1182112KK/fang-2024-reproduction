"""Independent numerical audit of Fang & Zhang, Automatica 159 (2024) 111399.

Source: supplied published PDF, equations (2)-(3), (35)-(40).
Only physical x is propagated. P is reconstructed from x and issued input
history at EVERY sample. No target system generates the physical trajectory.
This is sampled/ZOH feedback, not an exact continuous-feedback implementation.
Run: python audit.py --out results    (CPU; NumPy, SciPy, Numba, Matplotlib).
"""
from __future__ import annotations
import argparse, hashlib, json, platform, time
from pathlib import Path
import numpy as np
from numba import njit

@njit(cache=True)
def rhs(a,b,v1,v2):
    return b, -a+np.sin(a)+b+v1+np.tanh(v1)+np.sin(v2)

@njit(cache=True)
def step(a,b,h,v1,v2,order=4):
    k1,l1=rhs(a,b,v1,v2)
    if order==1:
        return a+h*k1,b+h*l1
    k2,l2=rhs(a+h*k1/2,b+h*l1/2,v1,v2)
    k3,l3=rhs(a+h*k2/2,b+h*l2/2,v1,v2)
    k4,l4=rhs(a+h*k3,b+h*l3,v1,v2)
    return a+h*(k1+2*k2+2*k3+k4)/6,b+h*(l1+2*l2+2*l3+l4)/6

@njit(cache=True)
def lookup(U,q,dt,k,ch):
    if q<0: return 0.
    j=int(np.floor(q/dt+1e-9))
    if j>k: raise ValueError('Noncausal input query')
    return U[j,ch]

@njit(cache=True)
def predict(a,b,U,k,dt,D0,order=4,substeps=1):
    """Eq. (3), bin-exact held history including a possible partial first bin."""
    lo=k*dt-D0
    j0=int(np.floor(lo/dt+1e-9))
    for j in range(j0,k):
        left=max(lo,j*dt); right=min(k*dt,(j+1)*dt)
        if right-left<=1e-14: continue
        v1=0. if j<0 else U[j,0]
        v2=0. if j<0 else U[j,1]
        h=(right-left)/substeps
        for _ in range(substeps):
            a,b=step(a,b,h,v1,v2,order)
    return a,b

@njit(cache=True)
def simulate_raw(dt,T,D0,D1,D2,mode=0,order=4,substeps=1,x10=1.,x20=1.):
    n=int(round(T/dt)); X=np.zeros((n+1,2)); U=np.zeros_like(X); P=np.zeros_like(X)
    X[0,0]=x10; X[0,1]=x20
    for k in range(n+1):
        a,b=X[k,0],X[k,1]
        if mode==0:
            p,q=predict(a,b,U,k,dt,D0,order,substeps)
        else: p,q=a,b
        P[k,0]=p; P[k,1]=q
        U[k,0]=-3*p-5*q; U[k,1]=-10*p-10*q
        if not np.isfinite(a+b+U[k,0]+U[k,1]) or abs(a)+abs(b)>1e12:
            raise FloatingPointError('Nonfinite state or explicit 1e12 safety bound')
        if k==n: break
        # Split physical RK4 intervals at every delayed held-command arrival.
        left=k*dt; end=(k+1)*dt
        while left<end-1e-13:
            next1=(np.floor((left-D1)/dt+1e-8)+1)*dt+D1
            next2=(np.floor((left-D2)/dt+1e-8)+1)*dt+D2
            right=min(end,next1,next2)
            if right<=left+1e-13: right=end
            mid=(left+right)/2
            v1=lookup(U,mid-D1,dt,k,0); v2=lookup(U,mid-D2,dt,k,1)
            a,b=step(a,b,right-left,v1,v2,order)
            left=right
        X[k+1,0]=a; X[k+1,1]=b
    return np.arange(n+1)*dt,X,U,P

def simulate(dt=.001,T=10.,D0=.45,D1=.4,D2=.5,mode='predictor',order=4,substeps=1):
    if not np.isfinite([dt,T,D0,D1,D2]).all() or dt<=0 or T<=0 or min(D0,D1,D2)<0:
        raise ValueError('Invalid parameters')
    if mode not in ('predictor','uncompensated','delay_free') or order not in (1,4):
        raise ValueError('Invalid mode/order')
    if abs(round(T/dt)*dt-T)>1e-9: raise ValueError('T must be an integer number of samples')
    if substeps<1 or int(substeps)!=substeps: raise ValueError('substeps must be a positive integer')
    if mode=='predictor' and D0<=0: raise ValueError('Predictor horizon must be positive')
    if mode=='delay_free': D1=D2=0.
    return simulate_raw(dt,T,D0,D1,D2,0 if mode=='predictor' else 1,order,substeps)

def summary(name,params,run):
    t,x,u,p=run
    return dict(case=name,**params,completed=True,t_end=float(t[-1]),
        final_x=x[-1].tolist(),final_norm=float(np.linalg.norm(x[-1])),
        initial_P=p[0].tolist(),initial_U=u[0].tolist(),
        maximum_sampled_state_norm=float(np.linalg.norm(x,axis=1).max()),
        minimum_sampled_inputs=u.min(axis=0).tolist(),maximum_sampled_inputs=u.max(axis=0).tolist())

def independent_checks():
    """Separate SciPy DOP853 startup and predictor time-derivative identity."""
    from scipy.integrate import solve_ivp
    def f(y,v): return np.array(rhs(y[0],y[1],v[0],v[1]))
    p0=solve_ivp(lambda s,y:f(y,[0,0]),(0,.45),[1,1],method='DOP853',rtol=2e-13,atol=2e-14).y[:,-1]
    # Manufactured smooth x(t), U(t); only an identity test, not a plant simulation.
    def xx(t): return np.array([.3*np.sin(t),.2*np.cos(2*t)])
    def xd(t): return np.array([.3*np.cos(t),-.4*np.sin(2*t)])
    def uu(s): return np.array([.2*np.sin(1.7*s),-.1*np.cos(.8*s)])
    def flow(t):
        def fun(s,y):
            p=y[:2]; J=np.array([[0.,1.],[-1+np.cos(p[0]),1.]])
            return np.r_[f(p,uu(s)),(J@y[2:].reshape(2,2)).ravel()]
        z=solve_ivp(fun,(t-.45,t),np.r_[xx(t),np.eye(2).ravel()],method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
        return z[:2],z[2:].reshape(2,2)
    t=.8; p,J=flow(t)
    exact=f(p,uu(t))+J@(xd(t)-f(xx(t),uu(t-.45)))
    defects=[]
    for eps in [1e-3,5e-4,2.5e-4]:
        fd=(flow(t+eps)[0]-flow(t-eps)[0])/(2*eps)
        defects.append(dict(fd_step=eps,max_abs_defect=float(np.max(np.abs(fd-exact)))))
    return dict(startup_P_DOP853=p0.tolist(),startup_U_DOP853=[float(-3*p0[0]-5*p0[1]),float(-10*sum(p0))],
                endpoint_identity='dP/dt=f(P,U(t))+Phi*(dx/dt-f(x,U(t-D0)))',
                manufactured_identity_defects=defects)

def make_plots(out,runs):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for name in ['predictor_h0.0005','uncompensated_h0.001','delay_free_h0.001']:
        t,x,u,p=runs[name]
        for y,kind in [(x,'state'),(u,'input')]:
            fig,ax=plt.subplots(figsize=(8,4.5))
            ax.plot(t,y[:,0],label=kind+' 1'); ax.plot(t,y[:,1],'--',label=kind+' 2')
            ax.set(xlabel='Time (s)',ylabel=kind,title=name.replace('_',' ')+' — '+kind)
            ax.legend(); ax.grid(True,alpha=.25); fig.tight_layout();fig.savefig(out/(name+'_'+kind+'.png'),dpi=160);plt.close(fig)
    t,x,u,p=runs['predictor_h0.0005']
    # Exact transport reconstruction for the issued ZOH commands, not a second PDE solver.
    z=np.linspace(0,1,81); tt=np.linspace(0,10,601); h=t[1]-t[0]
    for ch,D in enumerate([.4,.5]):
        query=tt[None,:]+D*(z[:,None]-1)
        ids=np.floor(query/h+1e-9).astype(int)
        field=np.where(query<0,0,u[np.clip(ids,0,len(t)-1),ch])
        np.savez_compressed(out/f'transport_{ch+1}.npz',t=tt,z=z,values=field)
        fig,ax=plt.subplots(figsize=(8,4.5));im=ax.pcolormesh(tt,z,field,shading='auto')
        ax.set(xlabel='Time (s)',ylabel='Normalized transport position z',title=f'Actuator {ch+1}: U{ch+1}(t+D{ch+1}(z−1))')
        fig.colorbar(im,ax=ax,label='Input');fig.tight_layout();fig.savefig(out/f'transport_{ch+1}.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4.5))
    for name in ['predictor_h0.002','predictor_h0.001','predictor_h0.0005','predictor_h0.00025']:
        t,x,_,_=runs[name];ax.semilogy(t,np.maximum(np.linalg.norm(x,axis=1),1e-16),label=name)
    ax.set(xlabel='Time (s)',ylabel='Euclidean physical-state norm',title='Sample-period refinement; RK4 + issued-command history')
    ax.legend();ax.grid(True,alpha=.25);fig.tight_layout();fig.savefig(out/'refinement.png',dpi=160);plt.close(fig)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('results'));ap.add_argument('--no-plots',action='store_true');args=ap.parse_args()
    out=args.out;out.mkdir(parents=True,exist_ok=True)
    cases=[]
    for h in [.002,.001,.0005,.00025]: cases.append((f'predictor_h{h}',dict(dt=h)))
    for mode in ['uncompensated','delay_free']:
        for h in [.002,.001,.0005]: cases.append((f'{mode}_h{h}',dict(dt=h,mode=mode)))
    for D in [.40,.42,.48,.50]: cases.append((f'horizon_{D}',dict(dt=.001,D0=D)))
    cases += [('equal_delays',dict(dt=.001,D1=.45,D2=.45)),
              ('long_horizon',dict(dt=.001,T=30.)),
              ('euler_comparison',dict(dt=.001,order=1)),
              ('offgrid_delays',dict(dt=.001,D1=.4003,D2=.4997,D0=.4502))]
    rows=[];runs={}
    for name,p in cases:
        start=time.perf_counter()
        try:
            r=simulate(**p);rows.append(summary(name,p,r));runs[name]=r
            np.savetxt(out/(name+'.csv'),np.column_stack(r),delimiter=',',header='t,x1,x2,u1,u2,P1,P2',comments='')
        except Exception as exc: rows.append(dict(case=name,**p,completed=False,error=repr(exc)))
        rows[-1]['wall_seconds']=time.perf_counter()-start
        print(json.dumps(rows[-1]),flush=True)
    finest=runs['predictor_h0.00025'];comp=[]
    for h in [.002,.001,.0005]:
        t,x,_,_=runs[f'predictor_h{h}'];ref=np.column_stack([np.interp(t,finest[0],finest[1][:,j]) for j in [0,1]])
        comp.append(dict(dt=h,max_componentwise_difference_to_h000025=float(np.max(abs(x-ref)))))
    eq=runs['equal_delays'];t,x,u,p=eq;mask=t+.45<=t[-1]
    xf=np.column_stack([np.interp(t[mask]+.45,t,x[:,j]) for j in [0,1]])
    checks=independent_checks();checks['equal_delay_sampled_prediction_max_error']=float(np.max(abs(xf-p[mask])))
    checks['refinement']=comp
    np.savez_compressed(out/'equal_delay_check.npz',t=t[mask],P=p[mask],future_physical_x=xf)
    import scipy,numba
    meta=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,numba=numba.__version__,
              source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              execution='Executed locally in the ChatGPT container; not Google Colab, MATLAB or GitHub CI',
              method='Physical RK4 split at held-input arrivals; predictor RK4 across exact held-history bins; sampled/ZOH feedback',
              source='Fang & Zhang 2024 DOI 10.1016/j.automatica.2023.111399; uploaded 9-page PDF')
    (out/'summary.json').write_text(json.dumps(rows,indent=2)+'\n');(out/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    (out/'provenance.json').write_text(json.dumps(meta,indent=2)+'\n')
    if not args.no_plots: make_plots(out,runs)

if __name__=='__main__': main()
