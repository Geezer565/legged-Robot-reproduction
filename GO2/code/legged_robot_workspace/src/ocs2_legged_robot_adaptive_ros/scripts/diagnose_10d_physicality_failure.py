#!/usr/bin/env python3
"""Read-only reconstruction of PayloadInertial10D combined-inertia validity."""
import json, os, sys
import numpy as np
import rosbag

ROOT=sys.argv[1]; M0=52.13485; TS=[20.,22.,24.,25.,25.5]
def pa(c): return np.dot(c,c)*np.eye(3)-np.outer(c,c)
def mat(p): return np.array([[p[4],p[5],p[6]],[p[5],p[7],p[8]],[p[6],p[8],p[9]]])
def physical(p,I0):
 m=float(p[0]); h=np.array(p[1:4]); J=mat(p); M=M0+m; c=h/M; I=I0+J-M*pa(c)
 e=np.linalg.eigvalsh((I+I.T)/2); pm=None; pe=None
 if abs(m)>1e-12:
  Ip=J-m*pa(h/m); pe=np.linalg.eigvalsh((Ip+Ip.T)/2); pm=Ip
 valid=bool(np.isfinite(I).all() and np.max(abs(I-I.T))<=1e-9 and e[0]>=-1e-9 and e[2]<=e[0]+e[1]+1e-9)
 return {"mass_hat":m,"total_mass":M,"h":h.tolist(),"total_com_shift":c.tolist(),"J":J.tolist(),"combined_I":I.tolist(),"combined_eigenvalues":e.tolist(),"combined_min_eigenvalue":float(e[0]),"combined_triangle_margin":float(e[0]+e[1]-e[2]),"payload_I_com":pm.tolist() if pm is not None else None,"payload_eigenvalues":pe.tolist() if pe is not None else None,"valid":valid}
def nearest(x,t): return min(x,key=lambda q:abs(q['time']-t))

# Recover the exact constant nominal inertia from C's successful model-info
# diagnostics: I0 = I_combined - J + M*P(h/M).
craw=[]
with rosbag.Bag(os.path.join(ROOT,'C','pilot.bag')) as b:
 for topic,msg,_ in b.read_messages(topics=['/task_d/payload_inertial_10d_sample']):
  d=list(msg.data); p=np.array(d[22:32]); M=d[12]; h=np.array(d[13:16]); Ic=np.array([[d[16],d[17],d[18]],[d[17],d[19],d[20]],[d[18],d[20],d[21]]]); craw.append((d[1],p,np.array(d[32:42]),d[42:102],d[102:108],d[0]))
I0s=[]
# At each successful C sample, flow inertia directly supplies I_combined.
with rosbag.Bag(os.path.join(ROOT,'C','pilot.bag')) as b:
 for _,msg,_ in b.read_messages(topics=['/task_d/payload_inertial_10d_sample']):
  d=list(msg.data); p=np.array(d[22:32]); M=M0+p[0]; h=p[1:4]; Ic=np.array([[d[16],d[17],d[18]],[d[17],d[19],d[20]],[d[18],d[20],d[21]]]); I0s.append(Ic-mat(p)+M*pa(h/M))
I0=np.median(np.stack(I0s),axis=0)

data={"nominal_mass":M0,"recovered_nominal_inertia":I0.tolist(),"modes":{}}
for mode in ('C','D'):
 snaps=[]
 path=os.path.join(ROOT,mode,'pilot.bag')
 with rosbag.Bag(path) as b:
  if mode=='C':
   for _,msg,_ in b.read_messages(topics=['/task_d/payload_inertial_10d_sample']):
    d=list(msg.data); snaps.append((d[1],np.array(d[22:32]),np.array(d[32:42]),np.array(d[102:108])))
  else:
   for _,msg,_ in b.read_messages(topics=['/adaptive_update/payload_inertial_plus_wrench_6d_sample']):
    d=list(msg.data); snaps.append((d[0],np.array(d[2:12]),None,np.array(d[36:42])))
 # v2 has no sigma. v3 inserts sigma at [42:48] while preserving v2 prefix.
 sigma_by_time={}
 if mode=='C':
  with rosbag.Bag(path) as sb:
   for _,msg,_ in sb.read_messages(topics=['/task_d/payload_inertial_10d_v3_sample']):
    d=list(msg.data); sigma_by_time[d[1]]=np.array(d[42:48])
 entries=[]
 for t,p,pdot,sigma in snaps:
  if mode=='C': sigma=sigma_by_time.get(t,sigma)
  q=physical(p,I0); q.update({"time":t,"pi":p.tolist(),"pi_dot_exact":pdot.tolist() if pdot is not None else None,"sigma":sigma.tolist()}); entries.append(q)
 # finite-difference pi_dot is also reported for C independently and is the
 # only available derivative for D's joint schema.
 for i in range(1,len(entries)):
  dt=entries[i]['time']-entries[i-1]['time']; entries[i]['pi_dot_fd']=((np.array(entries[i]['pi'])-np.array(entries[i-1]['pi']))/dt).tolist() if dt>0 else None
 entries[0]['pi_dot_fd']=None
 sampled=[]
 for t in TS: sampled.append(nearest(entries,t))
 last=entries[-1]
 dt=entries[-1]['time']-entries[-2]['time']; drive=np.array(last['pi_dot_exact'] if last['pi_dot_exact'] is not None else last['pi_dot_fd'])
 predicted=physical(np.array(last['pi'])+dt*drive,I0); predicted.update({"time":last['time']+dt,"pi":(np.array(last['pi'])+dt*drive).tolist(),"from_last_dt":dt})
 first_bad=next((x for x in entries if not x['valid']),None)
 data['modes'][mode]={"requested_times":sampled,"last_valid":last,"first_recorded_invalid":first_bad,"one_step_predicted":predicted}
with open(os.path.join(ROOT,'physicality_diagnosis.json'),'w') as f: json.dump(data,f,indent=2)
print(json.dumps(data,indent=2))
