#!/usr/bin/env python3
"""Optional offline regression: independent SciPy SLSQP solution of the full QP."""
import subprocess
import sys
import numpy as np
from scipy.optimize import LinearConstraint, minimize

text=subprocess.check_output([sys.argv[1], '--dump'],text=True)
data={line.split()[0]:np.array([float(v) for v in line.split()[1:]])
      for line in text.splitlines() if line.split()[0] in {'A','B','Q','R','X','REF','PARAM','U'}}
a,b=data['A'].reshape(4,4),data['B'].reshape(4,2)
hover,nominal,lo,hi,step,terminal=data['PARAM']
fx=np.zeros((64,4));fu=np.zeros((64,30));fx[:4]=np.eye(4)
for k in range(15):
    fx[4*(k+1):4*(k+2)]=a@fx[4*k:4*(k+1)]
    fu[4*(k+1):4*(k+2)]=a@fu[4*k:4*(k+1)]
    fu[4*(k+1):4*(k+2),2*k:2*(k+1)]=b
q=np.diag(np.tile(data['Q'],16));q[-4:,-4:]*=terminal
r=np.diag(np.tile(data['R'],15))
x=data['X'].copy();ref=data['REF'].copy();x[2]-=nominal;ref[2]-=nominal
h=2*(fu.T@q@fu+r);g=2*fu.T@q@(fx@x-np.tile(ref,16))
d=np.eye(30)
for i in range(2,30):d[i,i-2]=-1
c=np.vstack([np.eye(30),d])
lower=np.r_[np.full(30,lo-hover),np.full(30,-step)]
upper=np.r_[np.full(30,hi-hover),np.full(30,step)]
actual=data['U']-hover
solution=minimize(lambda u:.5*u@h@u+g@u,np.zeros(30),jac=lambda u:h@u+g,
                  constraints=[LinearConstraint(c,lower,upper)],method='SLSQP',
                  options={'maxiter':2000,'ftol':1e-10})
assert solution.success,solution.message
difference=float(abs(solution.x-actual).max())
objective_difference=abs((.5*actual@h@actual+g@actual)-solution.fun)
assert difference<.01,(difference,objective_difference)
assert np.maximum(lower-c@actual,c@actual-upper).max()<1e-3
print(f'PASS independent SLSQP: max force difference {difference:.6g} N; objective difference {objective_difference:.6g}')
