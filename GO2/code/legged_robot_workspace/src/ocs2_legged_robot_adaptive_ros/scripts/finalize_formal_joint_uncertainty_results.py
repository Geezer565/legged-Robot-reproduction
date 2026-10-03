#!/usr/bin/env python3
"""Read-only final tables, physicality audit, and refined formal figures."""
import csv, json, math, os, sys
import numpy as np, rosbag
import matplotlib.pyplot as plt
ROOT=sys.argv[1]; REPS=('repeat1','repeat2','repeat3'); M0=52.13485
def pa(c):return np.dot(c,c)*np.eye(3)-np.outer(c,c)
def mat(p):return np.array([[p[4],p[5],p[6]],[p[5],p[7],p[8]],[p[6],p[8],p[9]]])
def safety(p,I0):
 M=M0+p[0];I=I0+mat(p)-M*pa(p[1:4]/M);e=np.linalg.eigvalsh((I+I.T)/2);return float(e[0]),float(e[0]+e[1]-e[2])
def pairs(rep,mode,topic,kind):
 p=os.path.join(ROOT,rep,mode,'pilot.bag');ow=[];ot=[];raw=[]
 with rosbag.Bag(p) as b:
  for top,msg,st in b.read_messages():
   if top=='/legged_robot_mpc_observation':ow.append(st.to_sec());ot.append(msg.time)
   elif top==topic:raw.append((st.to_sec(),list(msg.data) if hasattr(msg.data,'__len__') else msg.data))
 return [(float(np.interp(w,ow,ot)),v) for w,v in raw]
def band(seq):
 g=np.linspace(0,20,1001);a=[np.interp(g,[x[0] for x in p],[x[1] for x in p]) for p in seq];return g,np.mean(a,0),np.std(a,0)
summary=json.load(open(os.path.join(ROOT,'formal_active_summary.json'))); per=summary['per_run']; meanstd=summary['mean_std']
metrics=('x_rmse','z_rmse','position_rmse','orientation_rmse','sigma_mean','sigma_rms')
with open(os.path.join(ROOT,'final_summary_table.csv'),'w',newline='') as f:
 w=csv.writer(f);w.writerow(['mode']+[x for k in metrics for x in (k+'_mean',k+'_std')])
 for q in meanstd:w.writerow([q['mode']]+[q[k+'_'+z] for k in metrics for z in ('mean','std')])
def active(mode):return next(q for q in meanstd if q['mode']==mode)
comparisons=(('wrench_6d','no_adaptation_clf'),('inertial_10d','no_adaptation_clf'),('joint_16d','no_adaptation_clf'),('joint_16d','inertial_10d'))
rel=[]
for a,b in comparisons:
 x,y=active(a),active(b);rel.append({'comparison':a+' vs '+b,**{k+'_reduction_pct':100*(y[k+'_mean']-x[k+'_mean'])/y[k+'_mean'] for k in metrics}})
with open(os.path.join(ROOT,'relative_improvement_table.json'),'w') as f:json.dump(rel,f,indent=2)
# Full-time C/D physicality: recover each repeat's nominal inertia from C flow samples.
audit={}
for rep in REPS:
 cs=[];ds=[]
 with rosbag.Bag(os.path.join(ROOT,rep,'C','pilot.bag')) as b:
  for top,msg,_ in b.read_messages(topics=['/task_d/payload_inertial_10d_sample']):
   d=list(msg.data);p=np.array(d[22:32]);M=M0+p[0];Ic=np.array([[d[16],d[17],d[18]],[d[17],d[19],d[20]],[d[18],d[20],d[21]]]);cs.append((d[1],p,Ic-mat(p)+M*pa(p[1:4]/M)))
 I0=np.median(np.stack([x[2] for x in cs]),axis=0)
 with rosbag.Bag(os.path.join(ROOT,rep,'D','pilot.bag')) as b:
  for top,msg,_ in b.read_messages(topics=['/adaptive_update/payload_inertial_plus_wrench_6d_sample']):
   d=list(msg.data);ds.append((d[0],np.array(d[2:12])))
 audit[rep]={}
 for mode,q in (('C',[(t,p) for t,p,_ in cs]),('D',ds)):
  vals=[(t,*safety(p,I0)) for t,p in q if 0<=t<=20]
  mn=min(vals,key=lambda x:x[1]);tm=min(vals,key=lambda x:x[2]);audit[rep][mode]={'min_eigenvalue':mn[1],'min_eigen_time':mn[0],'triangle_margin':tm[2],'triangle_margin_time':tm[0]}
agg={}
for mode in ('C','D'):
 agg[mode]={k:{'mean':float(np.mean([audit[r][mode][k] for r in REPS])),'std':float(np.std([audit[r][mode][k] for r in REPS],ddof=1))} for k in ('min_eigenvalue','triangle_margin')}
with open(os.path.join(ROOT,'physicality_full_window_audit.json'),'w') as f:json.dump({'per_repeat':audit,'mean_std':agg},f,indent=2)
# Refined Figure 4/5 and two responsibility panels.
def plot_scalar(modes,topic,label,file,true=None):
 plt.figure(figsize=(8,4.5))
 for m in modes:
  seq=[pairs(r,m,topic,'') for r in REPS];g,y,s=band(seq);plt.plot(g,y,label=m);plt.fill_between(g,y-s,y+s,alpha=.16)
 plt.axvspan(4,18,color='gray',alpha=.13,label='disturbance active')
 if true is not None:plt.hlines(true,4,18 if label.startswith('Fx') else 20,colors='k',linestyles='--',label='true value')
 plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(label);plt.grid();plt.legend();plt.tight_layout();plt.savefig(os.path.join(ROOT,file),dpi=160);plt.close()
plot_scalar(('B','D'),'/adaptive_plot/Fx_hat','Fx_hat [N]','figure4_fx_hat_refined.png',30)
plot_scalar(('C','D'),'/adaptive_plot/mass_hat','mass_hat [kg]','figure5_mass_hat_refined.png',5.213485)
for axis,idx,file in (('Fx',18,'figure6a_joint_wrench_Fx.png'),('Fz',20,'figure6b_joint_wrench_Fz.png')):
 plt.figure(figsize=(8,4.5))
 for label,off in (('inertial',0),('constant',6),('combined',12)):
  seq=[]
  for r in REPS:
   p=pairs(r,'D','/adaptive_update/payload_inertial_plus_wrench_6d_sample','');seq.append([(t,v[idx+off]) for t,v in p])
  g,y,s=band(seq);plt.plot(g,y,label=label);plt.fill_between(g,y-s,y+s,alpha=.16)
 plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(axis+' wrench [N]');plt.grid();plt.legend();plt.tight_layout();plt.savefig(os.path.join(ROOT,file),dpi=160);plt.close()
