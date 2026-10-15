#!/usr/bin/env python3
"""Read-only Phase-2 advisor figures from recorded_validated CSV files."""
import csv, json, importlib.util
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

ROOT=Path(__file__).resolve().parents[6]
spec=importlib.util.spec_from_file_location('p',Path(__file__).with_name('plot_go2_method_comparison.py'))
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
OUT=ROOT/'experi/results/phase2_generalization/advisor_figures_current'
CASES={
'x':{'Pure MPC':ROOT/'experi/results/phase1_5_xforce/20261004T174833.104926Z_10540ca3/advisor_force__flat_velocity_fx_constant__pure_mpc_standard__r001','Direct Wrench':ROOT/'experi/results/phase1_5_xforce/20261004T175022.022252Z_45e78910/advisor_force__flat_velocity_fx_constant__direct_wrench_standard__r001','Current RBF':ROOT/'experi/results/phase1_5_xforce/20261004T175449.478389Z_54ad7897/advisor_force__flat_velocity_fx_constant__v11_equivalent_wrench_standard__r001','RBF Candidate 01':ROOT/'experi/results/phase2_generalization/candidate_01_gamma_fy_2x/20261004T182300.116608Z_ce8df96c/advisor_force__flat_velocity_fx_constant__v11_equivalent_wrench_standard__r001'},
'y':{'Pure MPC':ROOT/'experi/results/phase2_generalization/current_y/20261004T181018.246217Z_4a1053f2/advisor_force__flat_velocity_fy_constant__pure_mpc_standard__r001','Direct Wrench':ROOT/'experi/results/phase2_generalization/current_y/20261004T181145.714018Z_788baaee/advisor_force__flat_velocity_fy_constant__direct_wrench_standard__r001','Current RBF':ROOT/'experi/results/phase2_generalization/current_y/20261004T181321.153396Z_7f65eae7/advisor_force__flat_velocity_fy_constant__v11_equivalent_wrench_standard__r001','RBF Candidate 01':ROOT/'experi/results/phase2_generalization/candidate_01_gamma_fy_2x/20261004T182650.820519Z_3f9876a2/advisor_force__flat_velocity_fy_constant__v11_equivalent_wrench_standard__r001'}}
C={'Pure MPC':'#666666','Direct Wrench':'#E69F00','Current RBF':'#0072B2','RBF Candidate 01':'#009E73'}
def nums(rows,k):return np.array([float(r[k]) for r in rows])
def load(path):
 _,s=p.read_rows(path/'state.csv');_,tar=p.read_rows(path/'mpc_target.csv'); mo=yaml.safe_load((path/'effective_rosparams.yaml').read_text())['go2_nominal_model'];t=nums(s,'wall_elapsed');ang=np.column_stack([nums(s,'observation_base_'+q) for q in ('yaw','pitch','roll')]);mom=np.column_stack([nums(s,'observation_normalized_momentum_'+str(i)) for i in range(6)]);a=p.physical_velocity(mom,p.rotation_zyx(ang),mo['mass'],np.array(mo['inertia_body_row_major']).reshape(3,3));ref=p.planner_on_state({'data':{'state':{'rows':s},'mpc_target':{'rows':tar}}});ta=np.column_stack([ref[q+'_rad'] for q in ('yaw','pitch','roll')]);tm=np.column_stack([ref[q] for q in p.LINEAR_TARGET_RATE_FIELDS+tuple('normalized_angular_momentum_'+q for q in 'xyz')]);r=p.physical_velocity(tm,p.rotation_zyx(ta),mo['mass'],np.array(mo['inertia_body_row_major']).reshape(3,3));return t,a,r,ang,nums(s,'observation_base_z'),s
def finish(fig,name):
 fig.tight_layout();fig.savefig(OUT/(name+'.png'),dpi=250);fig.savefig(OUT/(name+'.pdf'));plt.close(fig)
def line(ax,title):ax.axvline(6,color='k',ls=':',lw=1);ax.set_title(title);ax.grid(alpha=.22);ax.set_xlim(0,20)
def vel(condition,axis,name,error=False):
 fig,ax=plt.subplots(figsize=(9,4.6));ref=None
 for label,path in CASES[condition].items():
  t,a,r,*_=load(path);ref=(t,r); y=a[:,axis]-r[:,axis] if error else a[:,axis];ax.plot(t,y,label=label,color=C[label])
 if not error:ax.plot(ref[0],ref[1][:,axis],'k--',lw=1.6,label='Reference')
 line(ax,('World +X 20 N' if condition=='x' else 'World +Y 20 N')+' disturbance');ax.set_xlabel('Time [s]');ax.set_ylabel(('Forward velocity vx [m/s]' if axis==0 else 'Lateral velocity vy [m/s]') if not error else ('vx - vx_ref [m/s]' if axis==0 else 'vy - vy_ref [m/s]'));ax.legend(ncol=2,fontsize=9);finish(fig,name)
def stability(condition,name):
 fig,axs=plt.subplots(4,1,sharex=True,figsize=(9,8));labs=['roll [rad]','pitch [rad]','vy [m/s]','body z [m]']
 for label,path in CASES[condition].items():
  t,a,r,ang,z,_=load(path)
  for ax,y in zip(axs,[ang[:,2],ang[:,1],a[:,1],z]):ax.plot(t,y,color=C[label],label=label)
 for ax,l in zip(axs,labs):line(ax,('X-force' if condition=='x' else 'Y-force')+' stability');ax.set_ylabel(l)
 axs[0].legend(ncol=2,fontsize=8);axs[-1].set_xlabel('Time [s]');finish(fig,name)
def yupi(condition,name,main=None):
 fig,axs=plt.subplots(2,3,sharex=True,figsize=(11,6)); axes=['fx','fy','fz','tx','ty','tz'];units=['N','N','N','Nm','Nm','Nm']
 for label in ('Current RBF','RBF Candidate 01'):
  rows=list(csv.DictReader((CASES[condition][label]/'advisor_analysis/processed_yupi.csv').open()));t=np.array([float(x['time']) for x in rows])
  for ax,q,u in zip(axs.flat,axes,units):
   ax.plot(t,[float(x[q+'_active_hat']) for x in rows],color=C[label],label=label)
   if label=='Current RBF':ax.plot(t,[float(x[q+'_true']) for x in rows],'k--',lw=1.4,label='Ground Truth')
   if main is None:ax.axhline(0,color='#666666',lw=.8,ls='-.',label='Pure MPC: zero compensation' if q==axes[0] else None)
 for ax,q,u in zip(axs.flat,axes,units):line(ax,('X-force' if condition=='x' else 'Y-force')+' '+q.upper());ax.set_ylabel(u)
 axs[0,0].legend(fontsize=7);fig.text(.5,.01,'Direct Wrench omitted: its WORLD/base-origin internal quantity has no verified BODY/overall-CoM/left-hand transform.',ha='center',fontsize=8);finish(fig,name)
def mainplot(condition,q,name):
 fig,ax=plt.subplots(figsize=(9,4.6));
 for label in ('Current RBF','RBF Candidate 01'):
  rows=list(csv.DictReader((CASES[condition][label]/'advisor_analysis/processed_yupi.csv').open()));t=np.array([float(x['time']) for x in rows]);ax.plot(t,[float(x[q+'_active_hat']) for x in rows],color=C[label],label=label); 
  if label=='Current RBF':ax.plot(t,[float(x[q+'_true']) for x in rows],'k--',lw=1.5,label='Ground Truth')
 ax.axhline(0,color='#666666',ls='-.',label='Pure MPC: zero compensation');line(ax,('X-force' if condition=='x' else 'Y-force')+' '+q.upper());ax.set_xlabel('Time [s]');ax.set_ylabel('N' if q.startswith('f') else 'Nm');ax.legend();finish(fig,name)
def metrics():
 out=[]
 for cond,items in CASES.items():
  for label,path in items.items():
   t,a,r,ang,z,s=load(path);m=(t>=6)&(t<20);late=(t>=16)&(t<20);e=a-r;d={'condition':cond,'method':label,'seed':4102,'vx_rmse':np.sqrt(np.nanmean(e[m,0]**2)),'vy_rmse':np.sqrt(np.nanmean(e[m,1]**2)),'peak_vx_error':np.nanmax(abs(e[m,0])),'peak_vy_error':np.nanmax(abs(e[m,1])),'late_vx_error':np.nanmean(e[late,0]),'late_vy_error':np.nanmean(e[late,1]),'max_abs_roll':np.nanmax(abs(ang[m,2])),'max_abs_pitch':np.nanmax(abs(ang[m,1])),'max_abs_yaw_rate':np.nanmax(abs(np.gradient(np.unwrap(ang[:,0]),t)[m])),'max_abs_vy':np.nanmax(abs(a[m,1])),'z_min':np.nanmin(z[m]),'z_max':np.nanmax(z[m])}
   if 'RBF' in label:
    rows=list(csv.DictReader((path/'advisor_analysis/processed_yupi.csv').open())); rr=[x for x in rows if 6<=float(x['time'])<20]; es=[]
    for q in ['fx','fy','fz','tx','ty','tz']:
     v=np.array([float(x[q+'_active_hat'])-float(x[q+'_true']) for x in rr]);d[q.upper()+'_rmse']=np.sqrt(np.mean(v*v));es.extend(v*v)
    d['6D_total_rmse']=np.sqrt(sum(es)/len(rr))
   out.append(d)
 keys=sorted({k for x in out for k in x});
 with (OUT/'phase2_current_metrics.csv').open('w',newline='') as f:w=csv.DictWriter(f,keys);w.writeheader();w.writerows(out)
def run():
 OUT.mkdir(parents=True,exist_ok=True);vel('x',0,'01_xforce_vx_tracking');vel('x',0,'02_xforce_vx_error',True);vel('y',1,'03_yforce_vy_tracking');vel('y',1,'04_yforce_vy_error',True);vel('y',0,'05_yforce_vx_tracking');yupi('x','06_xforce_yupi_comparison');yupi('y','07_yforce_yupi_comparison');mainplot('x','fx','08_xforce_Fx_main');mainplot('x','ty','09_xforce_Ty_main');mainplot('y','fy','10_yforce_Fy_main');mainplot('y','tx','11_yforce_Tx_main');stability('x','12_xforce_stability');stability('y','13_yforce_stability');metrics()
if __name__=='__main__':run()
