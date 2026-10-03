#!/usr/bin/env python3
"""Aggregate frozen formal runs; read-only bag processing and six figures."""
import csv,json,math,os,sys
import numpy as np, rosbag
import matplotlib.pyplot as plt
ROOT=sys.argv[1]; repeats=('repeat1','repeat2','repeat3'); modes=('A','B','C','D'); names={'A':'no_adaptation_clf','B':'wrench_6d','C':'inertial_10d','D':'joint_16d'}
def wrap(x): return math.atan2(math.sin(x),math.cos(x))
def load(rep,m):
 p=os.path.join(ROOT,rep,m,'pilot.bag'); ow=[];ot=[];oq=[];target=None; raw={'sigma':[],'fx':[],'mass':[],'joint':[]}
 with rosbag.Bag(p) as b:
  for topic,msg,st in b.read_messages():
   w=st.to_sec()
   if topic=='/legged_robot_mpc_target': target=(list(msg.timeTrajectory),[list(x.value) for x in msg.stateTrajectory])
   elif topic=='/legged_robot_mpc_observation' and len(msg.state.value)>=12: ow.append(w);ot.append(msg.time);oq.append(list(msg.state.value))
   elif topic=='/adaptive_plot/sigma_norm': raw['sigma'].append((w,msg.data))
   elif topic=='/adaptive_plot/Fx_hat': raw['fx'].append((w,msg.data))
   elif topic=='/adaptive_plot/mass_hat': raw['mass'].append((w,msg.data))
   elif topic=='/adaptive_update/payload_inertial_plus_wrench_6d_sample': raw['joint'].append(list(msg.data))
 tt,ts=target
 def ref(t):
  if t<=tt[0]:return ts[0]
  if t>=tt[-1]:return ts[-1]
  a=(tt[1]-t)/(tt[1]-tt[0]);return[a*x+(1-a)*y for x,y in zip(ts[0],ts[1])]
 out={'pos':[],'z':[]}
 for t,q in zip(ot,oq):
  r=ref(t); e=[r[i]-q[i] for i in (6,7,8)];out['pos'].append((t,math.sqrt(sum(x*x for x in e))));out['z'].append((t,e[2]))
 for k in ('sigma','fx','mass'):out[k]=[(float(np.interp(w,ow,ot)),v) for w,v in raw[k]]
 out['joint']=raw['joint'];return out
S={m:[load(r,m) for r in repeats] for m in modes}
def band(ch,chosen):
 grid=np.linspace(0,20,1001); a=[]
 for s in chosen:a.append(np.interp(grid,[x[0] for x in s[ch]],[x[1] for x in s[ch]]))
 return grid,np.mean(a,0),np.std(a,0)
def fig(ch,mode_set,ylabel,file):
 plt.figure(figsize=(8,4.5))
 for m in mode_set:
  x,y,z=band(ch,S[m]);plt.plot(x,y,label=names[m]);plt.fill_between(x,y-z,y+z,alpha=.18)
 plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(ylabel);plt.grid();plt.legend();plt.tight_layout();plt.savefig(os.path.join(ROOT,file),dpi=160);plt.close()
fig('pos',modes,'position error norm [m]','figure1_position_error_norm.png');fig('z',modes,'z error [m]','figure2_z_error.png');fig('sigma',modes,'sigma norm','figure3_sigma_norm.png');fig('fx',('B','D'),'Fx_hat [N]','figure4_fx_hat.png');fig('mass',('C','D'),'mass_hat [kg]','figure5_mass_hat.png')
plt.figure(figsize=(8,4.5))
for label,idx in (('inertial Fx',18),('constant Fx',24),('combined Fx',30)):
 grid=np.linspace(0,20,1001); a=[]
 for s in S['D']:
  p=[(d[0],d[idx]) for d in s['joint']];a.append(np.interp(grid,[x[0] for x in p],[x[1] for x in p]))
 y=np.mean(a,0);plt.plot(grid,y,label=label)
plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel('wrench Fx [N]');plt.grid();plt.legend();plt.tight_layout();plt.savefig(os.path.join(ROOT,'figure6_joint_wrench_decomposition.png'),dpi=160);plt.close()
allruns=[]
for r in repeats:
 d=json.load(open(os.path.join(ROOT,r,'formal_candidate_analysis.json')))
 for q in d['window_metrics']:
  if q['window']=='active':q=dict(q);q['repeat']=r;allruns.append(q)
metrics=('x_rmse','z_rmse','position_rmse','orientation_rmse','sigma_mean','sigma_rms','sigma_max')
summary=[]
for m in names.values():
 q=[x for x in allruns if x['mode']==m];summary.append({'mode':m,**{k+'_mean':float(np.mean([x[k] for x in q])) for k in metrics},**{k+'_std':float(np.std([x[k] for x in q],ddof=1)) for k in metrics}})
with open(os.path.join(ROOT,'formal_active_summary.json'),'w') as f:json.dump({'per_run':allruns,'mean_std':summary},f,indent=2)
with open(os.path.join(ROOT,'formal_active_summary.csv'),'w',newline='') as f:w=csv.DictWriter(f,fieldnames=summary[0].keys());w.writeheader();w.writerows(summary)
