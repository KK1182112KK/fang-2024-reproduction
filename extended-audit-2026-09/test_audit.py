import numpy as np
import pytest
from scipy.integrate import solve_ivp
import audit
import continuous_check as cc

@pytest.mark.parametrize('h',[.005,.002,.001])
def test_original_unforced_prefix(h):
    t,x,u,p=audit.simulate(dt=h,T=.4)
    ref=solve_ivp(lambda t,y:audit.rhs(y[0],y[1],0,0),(0,.4),[1,1],method='DOP853',rtol=1e-12,atol=1e-13,dense_output=True)
    assert np.max(abs(x-ref.sol(t).T))<2e-10

@pytest.mark.parametrize('h',[.005,.001])
def test_initial_predictor_matches_independent_ivp(h):
    t,x,u,p=audit.simulate(dt=h,T=.1)
    ref=solve_ivp(lambda t,y:audit.rhs(y[0],y[1],0,0),(0,.45),[1,1],method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
    assert np.max(abs(p[0]-ref))<1e-10
    assert abs(u[0,1]+29.3576817206423)<1e-9

def test_equal_delays_true_physical_predictor():
    t,x,u,p=audit.simulate(dt=.002,T=2.,D1=.45,D2=.45)
    valid=t+.45<=2
    future=np.column_stack([np.interp(t[valid]+.45,t,x[:,j]) for j in [0,1]])
    assert np.max(abs(future-p[valid]))<1e-8

def test_terminal_commands_are_recomputed():
    t,x,u,p=audit.simulate(dt=.002,T=1.)
    assert np.allclose(u[:,0],-3*p[:,0]-5*p[:,1],atol=1e-13)
    assert np.allclose(u[:,1],-10*p[:,0]-10*p[:,1],atol=1e-13)

def test_no_future_horizon_dependence():
    short=audit.simulate(dt=.002,T=1.)
    long=audit.simulate(dt=.002,T=2.)
    for a,b in zip(short,long):assert np.array_equal(a,b[:len(a)])

def test_noncommensurate_delay_independent_dop853():
    h=.01;D1=.403;D2=.507
    t,x,u,p=audit.simulate(dt=h,T=.8,D1=D1,D2=D2,D0=.451)
    arrivals=np.unique(np.r_[t,t+D1,t+D2]);arrivals=arrivals[(arrivals>=0)&(arrivals<=.8)]
    y=np.array([1.,1.])
    for a,b in zip(arrivals[:-1],arrivals[1:]):
        if b-a<1e-13:continue
        q=(a+b)/2
        vals=[0. if q-D<0 else u[int(np.floor((q-D)/h+1e-9)),ch] for ch,D in enumerate([D1,D2])]
        y=solve_ivp(lambda t,y:audit.rhs(y[0],y[1],*vals),(a,b),y,method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
    assert max(abs(y-x[-1]))<1e-7

def test_continuous_fixed_point_and_unforced_prefix():
    t,x,u,p,err,it=cc.simulate(dt=.002,T=.4)
    ref=solve_ivp(lambda t,y:audit.rhs(y[0],y[1],0,0),(0,.4),[1,1],method='DOP853',rtol=1e-12,atol=1e-13).y[:,-1]
    assert max(abs(x[-1]-ref))<1e-10
    assert err<1e-9

def test_continuous_refinement_reduces_full_trajectory_error():
    runs=[cc.simulate(dt=h,T=2.) for h in [.005,.002,.001]]
    errors=[]
    for t,x,*_ in runs[:-1]:
        ref=np.column_stack([np.interp(t,runs[-1][0],runs[-1][1][:,j]) for j in [0,1]])
        errors.append(np.max(abs(x-ref)))
    assert errors[1]<errors[0]/3

def test_differentiated_predictor_identity():
    r=audit.independent_checks()['manufactured_identity_defects']
    assert r[-1]['max_abs_defect']<r[0]['max_abs_defect']/10

def test_positive_characteristic_roots_not_only_augmented_artifact():
    r=cc.spectral_audit()['roots']
    assert float(r[0]['real'])>.68 and abs(float(r[0]['imag'])-53.148865919)<1e-7
    assert float(r[0]['original_plant_and_predictor_eigenmode_residual'])<1e-55

@pytest.mark.parametrize('params',[dict(dt=0),dict(dt=-1),dict(mode='target'),dict(D0=-1)])
def test_invalid_parameters_fail(params):
    with pytest.raises(ValueError):audit.simulate(**params)
