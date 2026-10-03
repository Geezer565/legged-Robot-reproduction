#!/usr/bin/env python3
"""Read-only emergency audit of frozen formal joint-uncertainty bags."""
import csv, json, math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

ROOT=Path('/home/han/ocs2_ws/records/joint_uncertainty_abcd/formal_30N_4_18_20s_3repeat')
OUT=ROOT/'emergency_acceptance_audit'; OUT.mkdir(exist_ok=True)
REPS=('repeat1','repeat2','repeat3'); TRUTH=np.array([5.213485,.26067425,0,0,.0130337125,0,0,.0130337125,0,.0130337125])

def interp(t, tx, x):
    x=np.asarray(x); return np.column_stack([np.interp(t,tx,x[:,j]) for j in range(x.shape[1])]) if x.ndim==2 else np.interp(t,tx,x)
def Rzyx(a):
    z,y,x=a; cz,sz=np.cos(z),np.sin(z); cy,sy=np.cos(y),np.sin(y); cx,sx=np.cos(x),np.sin(x)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])
def cross(a,b): return np.cross(a,b)
def yin(R,omega,al,wr,ar):
    # Exact PayloadInertial10D::referenceRegressor algebra; recorded reference fields are controller fields.
    Y=np.zeros((6,10)); eff=al-np.array([0,0,-9.81]); Y[:3,0]=eff
    for i in range(3):
        h=R[:,i]; Y[:3,1+i]=cross(ar,h)+cross(omega,cross(wr,h)); Y[3:,1+i]=cross(h,eff)
    bases=[]
    for i,j in ((0,0),(0,1),(0,2),(1,1),(1,2),(2,2)):
        B=np.zeros((3,3)); B[i,j]=B[j,i]=1
        bases.append(B)
    for k,B in enumerate(bases):
        I=R@B@R.T; Y[3:,4+k]=I@ar+cross(omega,I@wr)
    return Y
def load(rep):
    p=ROOT/rep/'D'/'pilot.bag'; joint=[]; ref=[]; obs=[]; clf={k:[] for k in ('h','v','linear','damp','contrib','resid')}
    topics=['/adaptive_update/payload_inertial_plus_wrench_6d_sample','/task_d/d5i1_reference_sample','/legged_robot_mpc_observation','/aclf_experiment/executed_h_clf','/aclf_experiment/executed_violation','/aclf_experiment/executed_linear_term','/aclf_experiment/executed_damping_term','/aclf_experiment/executed_contribution','/aclf_experiment/executed_residual_wrench']
    with rosbag.Bag(str(p)) as b:
      for top,m,_ in b.read_messages(topics=topics):
        if top.endswith('plus_wrench_6d_sample'): joint.append(np.array(m.data,float))
        elif top.endswith('reference_sample'): ref.append(np.array(m.data,float))
        elif top.endswith('mpc_observation'): obs.append((float(m.time),np.array(m.state.value,float)))
        else:
          key={'/aclf_experiment/executed_h_clf':'h','/aclf_experiment/executed_violation':'v','/aclf_experiment/executed_linear_term':'linear','/aclf_experiment/executed_damping_term':'damp','/aclf_experiment/executed_contribution':'contrib','/aclf_experiment/executed_residual_wrench':'resid'}[top]; clf[key].append((float(m.header.stamp.to_sec()) if hasattr(m,'header') else 0., np.array(m.data if hasattr(m,'data') and not isinstance(m.data,float) else [m.data],float)))
    J=np.array(joint); F=np.array(ref); ot=np.array([x[0] for x in obs]); S=np.array([x[1] for x in obs]);
    # joint[0] is snapshot MPC time. reference includes the exact controller modified-reference kinematics.
    t=J[:,0]; eul=interp(t,ot,S[:,9:12]); Rs=np.array([Rzyx(q) for q in eul])
    # Actual omega is reconstructed from measured orientation; central finite difference on bag's ~100 Hz execution grid.
    # This is only needed in bilinear angular terms; all translational and h x g terms are exact controller fields.
    dt=np.gradient(t); omega=np.gradient(eul,axis=0)/dt[:,None]
    # mapping fields: [time,snapshot,error,ver,flag, desired p3, q4, desired v3,a3,w3,alpha3, vr3,ar3,wr3,alphar3, mode,phase,start]
    # The reference publisher can miss its first callback; synchronize it to
    # the immutable joint snapshot by its recorded snapshot time (field 1).
    al=interp(t,F[:,1],F[:,24:27]); wr=interp(t,F[:,1],F[:,33:36]); ar=interp(t,F[:,1],F[:,36:39])
    Y=np.array([yin(Rs[i],omega[i],al[i],wr[i],ar[i]) for i in range(len(t))])
    wh_in=np.einsum('nij,nj->ni',Y,J[:,2:12]); wt_in=np.einsum('nij,j->ni',Y,TRUTH)
    wf=np.zeros_like(wt_in); wf[(t>=4)&(t<=18),0]=30.; wt=wt_in+wf; wh=J[:,30:36]
    return t,wh,wt,wh_in,J[:,18:24],Y,clf
def window_stats(t,e):
    out={}
    for name,mask in [('pre',(t>=0)&(t<4)),('active',(t>=4)&(t<=18)),('post',(t>18)&(t<=20)),('4_7',(t>=4)&(t<7)),('7_12',(t>=7)&(t<12)),('12_18',(t>=12)&(t<=18))]:
      x=e[mask]; out[name]={'mean':float(x.mean()),'rms':float(np.sqrt(np.mean(x*x))),'final':float(x[-1])}
    return out
def band_plot(series,index,name,ylabel):
    g=np.linspace(0,20,1001); A=np.array([interp(g,x[0],x[1][:,index]) for x in series]); m=A.mean(0); s=A.std(0,ddof=1)
    plt.figure(figsize=(9,4)); plt.plot(g,m,label='estimated'); plt.fill_between(g,m-s,m+s,alpha=.2); B=np.array([interp(g,x[0],x[2][:,index]) for x in series]); plt.plot(g,B.mean(0),'k--',label='true'); plt.fill_between(g,B.mean(0)-B.std(0,ddof=1),B.mean(0)+B.std(0,ddof=1),color='k',alpha=.1); plt.axvspan(4,18,color='grey',alpha=.12);plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(ylabel);plt.grid();plt.legend();plt.tight_layout();plt.savefig(OUT/name,dpi=180);plt.close()
def main():
    runs=[load(r) for r in REPS]; report={'truth_pi_in':TRUTH.tolist(),'per_repeat':{},'regressor_reconstruction_note':'Y is reconstructed with PayloadInertial10D algebra and recorded controller modified-reference fields; angular velocity is finite-differenced from recorded executed orientation.'}
    for rep,x in zip(REPS,runs):
      t,wh,wt,win,winstored,Y,clf=x; eps=np.linalg.norm(wh-wt,axis=1); report['per_repeat'][rep]=window_stats(t,eps);report['per_repeat'][rep]['hat_in_regressor_vs_recorded_max_abs']=float(np.max(np.abs(win-winstored)))
    band_plot(runs,0,'U1_true_vs_estimated_Fx.png','Fx [N]'); band_plot(runs,2,'U2_true_vs_estimated_Fz.png','Fz [N]')
    g=np.linspace(0,20,1001); A=np.array([interp(g,x[0],np.linalg.norm(x[1]-x[2],axis=1)) for x in runs]);plt.figure(figsize=(9,4));plt.plot(g,A.mean(0));plt.fill_between(g,A.mean(0)-A.std(0,ddof=1),A.mean(0)+A.std(0,ddof=1),alpha=.2);plt.axvspan(4,18,color='grey',alpha=.12);plt.xlabel('MPC time [s]');plt.ylabel('||w_hat-w_true|| [N,Nm]');plt.xlim(0,20);plt.grid();plt.tight_layout();plt.savefig(OUT/'U3_epsilon_norm.png',dpi=180);plt.close()
    # Executed topics are scalar/vector separate but share message count and ordering from monitor callback.
    clfout={}
    for rep,x in zip(REPS,runs):
      _,_,_,_,_,_,c=x
      # ROS std_msgs have no header; recover MPC time by sample order against joint snapshot (same 100Hz monitor ordering).
      t=x[0]; h=np.array([z[1][0] for z in c['h']]); n=min(len(t),len(h)); tc=t[:n];h=h[:n];v=np.maximum(0,-h)
      manual=[]
      for k in range(n):
       co=c['contrib'][k][1]; da=c['damp'][k][1][0]; manual.append(float(np.sum(co)-da))
      manual=np.array(manual); picks={str(q):int(np.argmin(abs(tc-q))) for q in (3,6,10,15)}
      clfout[rep]={'manual_recorded_max_abs':float(np.max(abs(manual-h))),'manual_samples':{q:{'time':float(tc[i]),'manual_h':float(manual[i]),'recorded_h':float(h[i]),'linear':float(np.sum(c['contrib'][i][1])),'damping':float(c['damp'][i][1][0])} for q,i in picks.items()},'full':{'fraction_negative':float(np.mean(h<0)),'mean_violation':float(v.mean()),'rms_violation':float(np.sqrt(np.mean(v*v))),'max_violation':float(v.max())},'active':{'fraction_negative':float(np.mean(h[(tc>=4)&(tc<=18)]<0)),'mean_violation':float(v[(tc>=4)&(tc<=18)].mean()),'rms_violation':float(np.sqrt(np.mean(v[(tc>=4)&(tc<=18)]**2))),'max_violation':float(v[(tc>=4)&(tc<=18)].max())}}
      if rep==REPS[0]: pass
    # CLF curves on shared grid
    H=[]
    for x in runs:
      hh=np.array([z[1][0] for z in x[6]['h']]);H.append(interp(g,x[0][:len(hh)],hh[:len(x[0])]))
    H=np.array(H)
    for data,name,label in ((H,'C1_h_clf.png','h_clf'),(np.maximum(0,-H),'C2_violation.png','max(0,-h_clf)')):
      plt.figure(figsize=(9,4));plt.plot(g,data.mean(0));plt.fill_between(g,data.mean(0)-data.std(0,ddof=1),data.mean(0)+data.std(0,ddof=1),alpha=.2);plt.axhline(0,color='k',lw=.8);plt.axvspan(4,18,color='grey',alpha=.12);plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(label);plt.grid();plt.tight_layout();plt.savefig(OUT/name,dpi=180);plt.close()
    report['clf']=clfout
    json.dump(report,open(OUT/'audit.json','w'),indent=2)
    with open(OUT/'epsilon_statistics.csv','w',newline='') as f:
      w=csv.writer(f);w.writerow(['repeat','window','mean','rms','final']);[w.writerow([r,k,*v.values()]) for r,d in report['per_repeat'].items() for k,v in d.items() if isinstance(v,dict)]
if __name__=='__main__': main()
