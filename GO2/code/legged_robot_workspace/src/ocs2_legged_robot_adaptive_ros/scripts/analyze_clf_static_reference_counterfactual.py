#!/usr/bin/env python3
"""Offline-only Eq.(11) static-v_d counterfactual for the Paper-Gamma bag."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

BAG=Path('/home/han/ocs2_ws/records/gamma_audit_paper_gain_16d/D_paper_gamma/pilot.bag')
OUT=BAG.parent.parent/'analysis'
J='/adaptive_update/payload_inertial_plus_wrench_6d_sample'; R='/aclf_experiment/executed_residual_wrench'; H='/aclf_experiment/executed_h_clf'; T='/legged_robot_mpc_target'

def nearest(stamps, values, query):
    stamps=np.asarray(stamps); values=np.asarray(values); idx=np.searchsorted(stamps,query); idx=np.clip(idx,1,len(stamps)-1)
    idx-=np.abs(stamps[idx-1]-query)<np.abs(stamps[idx]-query)
    return values[idx],np.abs(stamps[idx]-query)
def stats(t,h,s,a,b):
    q=(t>=a)&(t<=b);x=h[q];v=np.maximum(0,-x)
    return {'samples':int(q.sum()),'min_h':float(x.min()),'mean_h':float(x.mean()),'fraction_h_negative':float((x<0).mean()),'mean_violation':float(v.mean()),'rms_violation':float(np.sqrt(np.mean(v*v))),'max_violation':float(v.max()),'max_sigma_norm':float(s[q].max())}

j=[];r=[];h=[];target=None
with rosbag.Bag(str(BAG)) as bag:
 for top,m,stamp in bag.read_messages(topics=[J,R,H,T]):
  if top==J:
   if len(m.data)!=44:raise RuntimeError('joint sample schema')
   j.append((stamp.to_sec(),np.asarray(m.data,float)))
  elif top==R:r.append((stamp.to_sec(),np.asarray(m.data,float)))
  elif top==H:h.append((stamp.to_sec(),float(m.data)))
  else:target=(np.asarray(m.timeTrajectory,float),[np.asarray(x.value,float) for x in m.stateTrajectory])
if not j or not r or not h or target is None:raise RuntimeError('missing required recorded topic')
j.sort();r.sort();h.sort();bs=np.array([x[0] for x in j]);x=np.vstack([z[1] for z in j]);t=x[:,0];sigma=x[:,36:42]
res,resdt=nearest([z[0] for z in r],[z[1] for z in r],bs);hcur,hdt=nearest([z[0] for z in h],[z[1] for z in h],bs)
if max(resdt.max(),hdt.max())>.02:raise RuntimeError('executed diagnostic pairing exceeds 20 ms')
# This run's only moving pose segment is read from the recorded target.  Its
# x secant is exactly the current v_d used by the pose-secant path; static
# legacy semantics set that v_d to zero while retaining all recorded Eq.(11)
# residual-wrench terms and K_D=I.
tt,ss=target;dx=ss[1][6]-ss[0][6];vdx=dx/(tt[1]-tt[0]);moving=(t>=tt[0])&(t<tt[1])
sigma_legacy=sigma.copy();sigma_legacy[moving,0]-=vdx
h_reconstructed=np.einsum('ij,ij->i',sigma,res)-.5*np.einsum('ij,ij->i',sigma,sigma)
h_legacy=np.einsum('ij,ij->i',sigma_legacy,res)-.5*np.einsum('ij,ij->i',sigma_legacy,sigma_legacy)
if np.max(np.abs(h_reconstructed-hcur))>1e-7:raise RuntimeError('recorded h is not reconstructed Eq.(11)')
s0=np.linalg.norm(sigma,axis=1);s1=np.linalg.norm(sigma_legacy,axis=1)
OUT.mkdir(exist_ok=True)
plt.figure(figsize=(9,4.5));plt.plot(t,hcur,label='current executed Eq.(11)');plt.plot(t,h_legacy,label='static-v_d counterfactual',ls='--');plt.axhline(0,color='k',lw=.8);plt.xlim(0,18);plt.xlabel('MPC time [s]');plt.ylabel('h_clf');plt.grid();plt.legend();plt.tight_layout();plt.savefig(OUT/'CLF_current_vs_legacy_counterfactual.png',dpi=180);plt.close()
plt.figure(figsize=(9,4.5));plt.plot(t,s0,label='current sigma norm');plt.plot(t,s1,label='static-v_d counterfactual',ls='--');plt.xlim(0,18);plt.xlabel('MPC time [s]');plt.ylabel('sigma norm');plt.grid();plt.legend();plt.tight_layout();plt.savefig(OUT/'SIGMA_current_vs_legacy_counterfactual.png',dpi=180);plt.close()
report={'counterfactual':'Only current pose-secant v_d,x=%.6g m/s on recorded target segment [%.6g,%.6g) is replaced by zero. Recorded residual wrench, K_D=I, state, estimate, and all other sigma components are retained.'%(vdx,tt[0],tt[1]),'current_reconstruction_max_abs_error':float(np.max(np.abs(h_reconstructed-hcur))),'segments':{}}
for n,a,b in [('0_0.5',0,.5),('2_10',2,10),('10_18',10,18)]:report['segments'][n]={'current':stats(t,hcur,s0,a,b),'legacy_counterfactual':stats(t,h_legacy,s1,a,b)}
report['target_transition_1.2_1.4']={'current':stats(t,hcur,s0,tt[0],tt[1]),'legacy_counterfactual':stats(t,h_legacy,s1,tt[0],tt[1])}
(OUT/'clf_static_reference_counterfactual.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
