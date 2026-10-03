#!/usr/bin/env python3
"""Read-only C/D screening analysis; requires the actual latched target in bag."""
import json, math, os, sys
import numpy as np
import rosbag

ROOT=sys.argv[1]; modes=("C","D"); names={"C":"inertial_10d","D":"joint_16d"}
def rms(x): return float(np.sqrt(np.mean(np.square(x)))) if x else None
def wrap(x): return math.atan2(math.sin(x),math.cos(x))
def sel(p): return [v for t,v in p if 4<=t<=24]
def stat(p):
 x=sel(p); return {"n":len(x),"mean":float(np.mean(x)),"rms":rms(x),"max":float(np.max(x))}
def near(p,t): return min(p,key=lambda x:abs(x[0]-t))[1]
S={}; report={}
for m in modes:
 obs_w=[]; obs_t=[]; obs_q=[]; target=None; raw={"sigma":[],"mass":[],"fx":[],"joint":[]}
 with rosbag.Bag(os.path.join(ROOT,m,"pilot.bag")) as b:
  for topic,msg,stamp in b.read_messages():
   w=stamp.to_sec()
   if topic=="/legged_robot_mpc_target": target=(list(msg.timeTrajectory),[list(q.value) for q in msg.stateTrajectory])
   elif topic=="/legged_robot_mpc_observation" and len(msg.state.value)>=12: obs_w.append(w);obs_t.append(msg.time);obs_q.append(list(msg.state.value))
   elif topic=="/adaptive_plot/sigma_norm": raw["sigma"].append((w,msg.data))
   elif topic=="/adaptive_plot/mass_hat": raw["mass"].append((w,msg.data))
   elif topic=="/adaptive_plot/Fx_hat": raw["fx"].append((w,msg.data))
   elif topic=="/adaptive_update/payload_inertial_plus_wrench_6d_sample": raw["joint"].append(list(msg.data))
 if target is None: raise RuntimeError("actual /legged_robot_mpc_target missing from "+m)
 tt,ts=target
 def ref(t):
  if t<=tt[0]: return ts[0]
  if t>=tt[-1]: return ts[-1]
  a=(tt[1]-t)/(tt[1]-tt[0]); return [a*x+(1-a)*y for x,y in zip(ts[0],ts[1])]
 e={k:[] for k in ("x","z","position","orientation")}
 for t,q in zip(obs_t,obs_q):
  r=ref(t); ep=[r[i]-q[i] for i in (6,7,8)]; eo=[wrap(r[i]-q[i]) for i in (11,10,9)]
  e["x"].append((t,ep[0]));e["z"].append((t,ep[2]));e["position"].append((t,math.sqrt(sum(v*v for v in ep))));e["orientation"].append((t,math.sqrt(sum(v*v for v in eo))))
 def tm(p): return [(float(np.interp(w,obs_w,obs_t)),v) for w,v in p]
 S[m]={"errors":e,"sigma":tm(raw["sigma"]),"mass":tm(raw["mass"]),"fx":tm(raw["fx"]),"joint":raw["joint"],"target":target}
 report[m]={k+"_rmse":rms(sel(p)) for k,p in e.items()}; report[m].update({"sigma_"+k:v for k,v in stat(S[m]["sigma"]).items()})
C,D=report["C"],report["D"]
report["D_vs_C_reduction_pct"]={k:100*(C[k]-D[k])/C[k] for k in ("x_rmse","z_rmse","position_rmse","orientation_rmse","sigma_mean","sigma_rms")}
p=S["D"]["fx"]; report["D_fx"]={"at_"+str(t):near(p,t) for t in (4,8,12,16,20,23.5)};report["D_fx"]["active"]=stat(p)
p=S["D"]["mass"]; report["D_mass"]={"at_"+str(t):near(p,t) for t in (4,8,12,16,20,23.5)}
for name,idx in (("inertial_Fx",18),("constant_Fx",24),("combined_Fx",30)):
 report["D_"+name]=stat([(x[0],x[idx]) for x in S["D"]["joint"] if len(x)==44])
report["target_recorded"]={m:{"times":S[m]["target"][0],"states":len(S[m]["target"][1])} for m in modes}
with open(os.path.join(ROOT,"screening_30N.json"),"w") as f: json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
