#!/usr/bin/env python3
"""Direct-recording comparison for the one-shot 16D Paper-Gamma audit."""
import json, sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

DIAG='/adaptive_diagnostics/exact_joint_uncertainty_wrench'; MASS='/adaptive_plot/mass_hat'
OBS='/legged_robot_mpc_observation'; TARGET='/legged_robot_mpc_target'; SIGMA='/aclf_experiment/executed_sigma_norm'
WINDOWS=(('pre',0,4,False),('active',4,18,True),('4_7',4,7,False),('7_12',7,12,False),('12_18',12,18,True),('post',18,20,True))

def interp(x, xp, fp): return np.interp(x, xp, fp)
def window(t,x):
 out={}
 for n,a,b,inc in WINDOWS:
  q=x[(t>=a)&((t<=b) if inc else (t<b))];out[n]={'mean':float(q.mean()),'rms':float(np.sqrt(np.mean(q*q))),'final':float(q[-1]),'samples':int(len(q))}
 return out
def desired(t,tt,ss):
 if t<=tt[0]:return np.asarray(ss[0])
 if t>=tt[-1]:return np.asarray(ss[-1])
 i=np.searchsorted(tt,t);a=(tt[i]-t)/(tt[i]-tt[i-1]);return a*np.asarray(ss[i-1])+(1-a)*np.asarray(ss[i])
def load(path):
 d=[];ma=[];ob=[];si=[];tar=None
 with rosbag.Bag(str(path)) as b:
  for top,m,s in b.read_messages(topics=[DIAG,MASS,OBS,TARGET,SIGMA]):
   if top==DIAG:
    if len(m.data)!=44:raise RuntimeError('diagnostic schema')
    d.append((s.to_sec(),np.asarray(m.data,float)))
   elif top==MASS:ma.append((s.to_sec(),float(m.data)))
   elif top==OBS:ob.append((s.to_sec(),float(m.time),np.asarray(m.state.value,float)))
   elif top==TARGET:tar=(np.asarray(m.timeTrajectory,float),[x.value for x in m.stateTrajectory])
   else:si.append((s.to_sec(),float(m.data)))
 if not d or not ma or not ob or not si or tar is None:raise RuntimeError('missing recorded audit topic')
 d.sort(); ob.sort();ma=np.asarray(ma);si=np.asarray(si); db=np.array([x[0] for x in d]);x=np.vstack([x[1] for x in d]); ot=np.array([x[1] for x in ob]);obag=np.array([x[0] for x in ob]);state=np.vstack([x[2] for x in ob])
 z=np.array([desired(t,*tar)[8]-s[8] for t,s in zip(ot,state)]);pos=np.array([np.linalg.norm(desired(t,*tar)[6:9]-s[6:9]) for t,s in zip(ot,state)])
 return {'t':x[:,0],'bag':db,'what':x[:,25:31],'wtrue':x[:,31:37],'eps':x[:,43],'mass_t':interp(ma[:,0],db,x[:,0]),'mass':ma[:,1],'ot':ot,'z':z,'pos':pos,'sigma_t':interp(si[:,0],obag,ot),'sigma':si[:,1]}
def active_metric(t,x):
 q=x[(t>=4)&(t<=18)];return {'mean':float(q.mean()),'rms':float(np.sqrt(np.mean(q*q))),'samples':int(len(q))}
def mass_stats(q):
 m=(q['mass_t']>=15)&(q['mass_t']<=20);t,x=q['mass_t'][m],q['mass'][m]
 return {'final':float(q['mass'][-1]),'max':float(q['mass'].max()),'last5_mean':float(x.mean()),'last5_std':float(x.std()),'last5_slope':float(np.polyfit(t,x,1)[0])}
def plot(series,y,out):
 plt.figure(figsize=(9,4.5));[plt.plot(t,x,label=n,linewidth=1.25) for n,t,x in series];plt.xlim(0,20);plt.xlabel('MPC time [s]');plt.ylabel(y);plt.grid();plt.legend();plt.tight_layout();plt.savefig(out,dpi=180);plt.close()
def main():
 if len(sys.argv)!=4:raise SystemExit('usage: analyze_gamma_audit_16d.py BASE PAPER OUT')
 b,p,out=load(Path(sys.argv[1])),load(Path(sys.argv[2])),Path(sys.argv[3]);out.mkdir(parents=True,exist_ok=True)
 for tag,q in [('baseline',b),('paperGamma',p)]:
  plot([('true',q['t'],q['wtrue'][:,0]),('hat',q['t'],q['what'][:,0])],'Fx [N]',out/f'U1_{tag}_Fx_true_vs_hat.png')
  plot([('true',q['t'],q['wtrue'][:,2]),('hat',q['t'],q['what'][:,2])],'Fz [N]',out/f'U2_{tag}_Fz_true_vs_hat.png')
  plot([('epsilon norm',q['t'],q['eps'])],'epsilon norm [N,Nm]',out/f'U3_{tag}_epsilon_norm.png')
 plot([('baseline',b['t'],b['eps']),('paper Gamma',p['t'],p['eps'])],'epsilon norm [N,Nm]',out/'G1_baseline_vs_paperGamma_epsilon_norm.png')
 plot([('baseline hat',b['t'],b['what'][:,2]),('paper Gamma hat',p['t'],p['what'][:,2]),('true',b['t'],b['wtrue'][:,2])],'Fz [N]',out/'G2_baseline_vs_paperGamma_Fz_hat.png')
 plot([('baseline',b['mass_t'],b['mass']),('paper Gamma',p['mass_t'],p['mass'])],'payload mass estimate [kg]',out/'G3_baseline_vs_paperGamma_mass_hat.png')
 R={}
 for n,q in [('baseline',b),('paperGamma',p)]:
  fz=q['what'][:,2]-q['wtrue'][:,2];m=(q['t']>=12)&(q['t']<=18)
  R[n] = {
   'epsilon_norm': window(q['t'], q['eps']),
   'Fz': {'active_RMSE': float(np.sqrt(np.mean(fz[(q['t'] >= 4) & (q['t'] <= 18)] ** 2))),
          '12_18_mean': float(fz[m].mean()), '12_18_RMS': float(np.sqrt(np.mean(fz[m] ** 2))),
          'final_hat': float(q['what'][-1, 2]), 'final_true': float(q['wtrue'][-1, 2])},
   'mass_hat': mass_stats(q),
   'tracking': {'z_RMSE': active_metric(q['ot'], q['z'])['rms'],
                'position_RMSE': active_metric(q['ot'], q['pos'])['rms'],
                'sigma_mean': active_metric(q['sigma_t'], q['sigma'])['mean'],
                'sigma_RMS': active_metric(q['sigma_t'], q['sigma'])['rms']}}
 (out/'gamma_audit_report.json').write_text(json.dumps(R,indent=2)+'\n');print(json.dumps(R,indent=2))
if __name__=='__main__':main()
