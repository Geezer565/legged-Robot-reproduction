#!/usr/bin/env python3
"""Read-only comparison of recorded solver-latest and executed Eq.(11) CLF."""
import json
from pathlib import Path
import numpy as np
import rosbag

B=Path('/home/han/ocs2_ws/records/gamma_audit_paper_gain_16d/D_paper_gamma/pilot.bag');O=B.parent.parent/'analysis'
E={'h':'/aclf_experiment/executed_h_clf','t':'/aclf_experiment/executed_observation_time','c':'/aclf_experiment/executed_contribution'}
S='/aclf/h_clf';MS='/legged_robot_mpc_mode_schedule'
def near(a,v,q):
 a=np.asarray(a);i=np.searchsorted(a,q);i=np.clip(i,1,len(a)-1);i-=np.abs(a[i-1]-q)<np.abs(a[i]-q);return np.asarray(v)[i],np.abs(a[i]-q)
def stat(h):
 v=np.maximum(0,-h);return {'samples':int(len(h)),'fraction_h_negative':float((h<0).mean()),'mean_violation':float(v.mean()),'rms_violation':float(np.sqrt(np.mean(v*v))),'max_violation':float(v.max())}
raw={k:[] for k in E};solver=[];schedule=None
with rosbag.Bag(str(B)) as bag:
 for top,m,stamp in bag.read_messages(topics=list(E.values())+[S,MS]):
  z=stamp.to_sec()
  if top==S:solver.append((z,float(m.data)))
  elif top==MS:schedule=(list(m.eventTimes),list(m.modeSequence))
  else:
   for k,n in E.items():
    if top==n:raw[k].append((z,np.asarray(m.data,float) if k=='c' else float(m.data)))
if not all(raw.values()) or not solver or schedule is None:raise RuntimeError('missing required bag data')
raw['t'].sort();anchors=np.array([z for z,_ in raw['t']]);times=np.array([v for _,v in raw['t']]);
# Associate h and contribution to their same timer callback by bag stamp.
h,he=near([z for z,_ in raw['h']],[v for _,v in raw['h']],anchors);c,ce=near([z for z,_ in raw['c']],[v for _,v in raw['c']],anchors)
keep=(he<=.01)&(ce<=.01);t=times[keep];h=h[keep];c=c[keep]
sh,se=near([z for z,_ in solver],[v for _,v in solver],anchors);st=times[se<=.02];sh=sh[se<=.02]
events, modes=schedule;period=events[-1]-events[0]
trans=np.array([e+k*period for k in range(16) for e in events if 0<=e+k*period<=18])
distance=np.min(np.abs(t[:,None]-trans[None,:]),axis=1);switch=distance<=.03
report={'solver_latest_semantics':'/aclf/h_clf is the 10 Hz timer readout of AdaptiveClfConstraint::diagnostics().latest, i.e. the latest nominal solver constraint evaluation; it is not a per-shooting-node trajectory log.', 'pairing_max_s':float(max(he[keep].max(),ce[keep].max())), 'solver_vs_executed':{},'contact_phase':{},'negative_peaks':[]}
for n,a,b in [('2_10',2,10),('10_18',10,18)]:
 report['solver_vs_executed'][n]={'solver_latest':stat(sh[(st>=a)&(st<=b)]),'executed':stat(h[(t>=a)&(t<=b)])}
for n,m in [('switch_pm_0.03',switch),('phase_interior',~switch)]:report['contact_phase'][n]=stat(h[m&(t>=2)&(t<=18)])
# List five most negative executed samples and their exact six-axis linear contributions.
for i in np.flatnonzero((t>=2)&(t<=18))[np.argsort(h[(t>=2)&(t<=18)])[:5]]:report['negative_peaks'].append({'time':float(t[i]),'h':float(h[i]),'near_contact_switch':bool(switch[i]),'contribution_Fx_Fy_Fz_Tx_Ty_Tz':[float(x) for x in c[i]]})
O.mkdir(exist_ok=True);(O/'solver_vs_executed_clf_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
