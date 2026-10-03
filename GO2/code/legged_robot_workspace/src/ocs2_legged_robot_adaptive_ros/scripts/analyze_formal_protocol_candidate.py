#!/usr/bin/env python3
"""Read-only metrics/safety audit for the fixed 20 s formal-protocol candidate."""
import json, math, os, sys
import numpy as np
import rosbag
ROOT=sys.argv[1]; MODES=('A','B','C','D'); N={'A':'no_adaptation_clf','B':'wrench_6d','C':'inertial_10d','D':'joint_16d'}; M0=52.13485
W={'full':(0,20),'pre':(0,4),'active':(4,18),'post':(18,20)}
def rms(x): return float(np.sqrt(np.mean(np.square(x)))) if x else None
def pa(c): return np.dot(c,c)*np.eye(3)-np.outer(c,c)
def mat(p): return np.array([[p[4],p[5],p[6]],[p[5],p[7],p[8]],[p[6],p[8],p[9]]])
def wrap(x): return math.atan2(math.sin(x),math.cos(x))
def stat(p,lo,hi):
 x=[v for t,v in p if lo<=t<=hi]; return {'mean':float(np.mean(x)) if x else None,'rms':rms(x),'max':float(np.max(x)) if x else None,'n':len(x)}
def near(p,t): return min(p,key=lambda q:abs(q[0]-t))[1]
S={}; rows=[]; targets={}; c_samples=[]
for m in MODES:
 ow=[]; ot=[]; oq=[]; target=None; raw={'sigma':[],'mass':[],'fx':[],'joint':[]}
 with rosbag.Bag(os.path.join(ROOT,m,'pilot.bag')) as b:
  for topic,msg,stamp in b.read_messages():
   wall=stamp.to_sec()
   if topic=='/legged_robot_mpc_target': target=(list(msg.timeTrajectory),[list(x.value) for x in msg.stateTrajectory])
   elif topic=='/legged_robot_mpc_observation' and len(msg.state.value)>=12: ow.append(wall);ot.append(msg.time);oq.append(list(msg.state.value))
   elif topic=='/adaptive_plot/sigma_norm': raw['sigma'].append((wall,msg.data))
   elif topic=='/adaptive_plot/mass_hat': raw['mass'].append((wall,msg.data))
   elif topic=='/adaptive_plot/Fx_hat': raw['fx'].append((wall,msg.data))
   elif topic=='/adaptive_update/payload_inertial_plus_wrench_6d_sample': raw['joint'].append(list(msg.data))
   elif m=='C' and topic=='/task_d/payload_inertial_10d_sample': c_samples.append(list(msg.data))
 if target is None: raise RuntimeError('missing actual target '+m)
 targets[m]=target; tt,ts=target
 def ref(t):
  if t<=tt[0]: return ts[0]
  if t>=tt[-1]: return ts[-1]
  a=(tt[1]-t)/(tt[1]-tt[0]); return [a*x+(1-a)*y for x,y in zip(ts[0],ts[1])]
 e={k:[] for k in ('x','z','position','orientation')}
 for t,q in zip(ot,oq):
  r=ref(t); ep=[r[i]-q[i] for i in (6,7,8)]; eo=[wrap(r[i]-q[i]) for i in (11,10,9)]
  e['x'].append((t,ep[0]));e['z'].append((t,ep[2]));e['position'].append((t,math.sqrt(sum(v*v for v in ep))));e['orientation'].append((t,math.sqrt(sum(v*v for v in eo))))
 def tm(x): return [(float(np.interp(w,ow,ot)),v) for w,v in x]
 S[m]={'e':e,'sigma':tm(raw['sigma']),'mass':tm(raw['mass']),'fx':tm(raw['fx']),'joint':raw['joint'],'end':ot[-1]}
 for win,(lo,hi) in W.items():
  r={'mode':N[m],'window':win}
  for k,p in e.items(): r[k+'_rmse']=rms([v for t,v in p if lo<=t<=hi])
  r.update({'sigma_'+k:v for k,v in stat(S[m]['sigma'],lo,hi).items()}); rows.append(r)
# Recover nominal inertia directly from C's valid full-flow output.
I0s=[]
for d in c_samples:
 p=np.array(d[22:32]); M=M0+p[0]; h=p[1:4]; Ic=np.array([[d[16],d[17],d[18]],[d[17],d[19],d[20]],[d[18],d[20],d[21]]]); I0s.append(Ic-mat(p)+M*pa(h/M))
I0=np.median(np.stack(I0s),axis=0)
def safety(p):
 M=M0+p[0]; c=p[1:4]/M; I=I0+mat(p)-M*pa(c); ev=np.linalg.eigvalsh((I+I.T)/2); return {'min_eigenvalue':float(ev[0]),'triangle_margin':float(ev[0]+ev[1]-ev[2])}
cs=[(d[1],np.array(d[22:32])) for d in c_samples]; ds=[(d[0],np.array(d[2:12])) for d in S['D']['joint'] if len(d)==44]
safe={'C':dict(safety(near(cs,20))),'D':dict(safety(near(ds,20)))}
active={r['mode']:r for r in rows if r['window']=='active'}
improve={}
for a,b in (('B','A'),('C','A'),('D','A'),('D','C')):
 x,y=active[N[a]],active[N[b]]; improve[N[a]+' vs '+N[b]]={k:100*(y[k]-x[k])/y[k] for k in ('x_rmse','z_rmse','position_rmse','orientation_rmse','sigma_mean','sigma_rms')}
djoint=[d for d in S['D']['joint'] if 4<=d[0]<=18]
def dstat(i): return stat([(d[0],d[i]) for d in djoint],4,18)
out={'target_recorded':{m:{'times':targets[m][0],'states':len(targets[m][1])} for m in MODES},'end_time':{m:S[m]['end'] for m in MODES},'window_metrics':rows,'active_improvements_pct':improve,'fx':{m:{'at4':near(S[m]['fx'],4),'at8':near(S[m]['fx'],8),'at12':near(S[m]['fx'],12),'at16':near(S[m]['fx'],16),'at18':near(S[m]['fx'],18),'active':stat(S[m]['fx'],4,18)} for m in ('B','D')},'mass':{m:{'at4':near(S[m]['mass'],4),'at8':near(S[m]['mass'],8),'at12':near(S[m]['mass'],12),'at16':near(S[m]['mass'],16),'at18':near(S[m]['mass'],18),'at20':near(S[m]['mass'],20)} for m in ('C','D')},'D_decomposition_active':{k:dstat(i) for k,i in {'inertial_Fx':18,'inertial_Fz':20,'constant_Fx':24,'constant_Fz':26,'combined_Fx':30,'combined_Fz':32}.items()},'safety_at_20':safe,'nominal_inertia':I0.tolist()}
with open(os.path.join(ROOT,'formal_candidate_analysis.json'),'w') as f: json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
