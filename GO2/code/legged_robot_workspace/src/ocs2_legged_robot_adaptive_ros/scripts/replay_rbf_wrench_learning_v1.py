#!/usr/bin/env python3
"""Offline-only replay of the frozen RbfWrenchNetwork adaptive law."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

ROOT=Path("/home/han/ocs2_ws/records/rbf_offline_learning_v1"); ROOT.mkdir(parents=True,exist_ok=True)
BASIS=Path("/home/han/ocs2_ws/records/rbf_basis_design")
CASES={"CaseFxP":Path("/home/han/ocs2_ws/records/rbf_residual_observer_20N/pilot.bag"),"Case0":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/Case0/pilot.bag"),"CaseFxN":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFxN/pilot.bag"),"CaseFyP":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFyP/pilot.bag"),"CaseFzP":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFzP/pilot.bag"),"CaseTxP":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTxP/pilot.bag"),"CaseTyP":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTyP/pilot.bag"),"CaseTzP":Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTzP/pilot.bag")}
GAMMAS=(.25,.5,1.,2.); BETAS=(1.,.7); TOL=.011; NAMES=("Fx","Fy","Fz","Tx","Ty","Tz")
R_TOPIC="/rbf_diagnostics/wrench_residual_filtered"; S_TOPIC="/adaptive_update/sample"; T_TOPIC="/adaptive_update/disturbance_sample"

def vector_wrench(m):
 w=m.wrench; return np.array([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z],float)
def nearest(t,v,q):
 i=np.searchsorted(t,q); k=min((x for x in (i-1,i) if 0<=x<len(t)),key=lambda x:abs(t[x]-q)); return v[k],abs(t[k]-q)
def norm_stats(x):
 x=np.asarray(x,float); return {"mean":float(x.mean()),"median":float(np.median(x)),"p05":float(np.percentile(x,5)),"p95":float(np.percentile(x,95)),"min":float(x.min()),"max":float(x.max())}
def rmse(error): return np.sqrt(np.mean(error**2,axis=0))

def read(case,path):
 st=[]; sv=[]; rr=[]; truth=[]
 with rosbag.Bag(str(path)) as b:
  for topic,m,_ in b.read_messages(topics=[R_TOPIC,S_TOPIC,T_TOPIC]):
   if topic==S_TOPIC and len(m.data)>=16: st.append(float(m.data[0])); sv.append(np.asarray(m.data[10:16],float))
   elif topic==R_TOPIC and 1.<=m.header.stamp.to_sec()<=20.: rr.append((m.header.stamp.to_sec(),vector_wrench(m)))
   elif topic==T_TOPIC and len(m.data)>=8: truth.append((float(m.data[0]),np.asarray(m.data[2:8],float)))
 st=np.asarray(st); sv=np.asarray(sv); order=np.argsort(st); st,sv=st[order],sv[order]
 rows=[]; errs=[]
 for time,residual in rr:
  sigma,e=nearest(st,sv,time)
  if e<=TOL: rows.append((time,residual,sigma));errs.append(e)
 return rows, {"bag_path":str(path),"raw_residual_1_to_20":len(rr),"aligned":len(rows),"dropped_alignment":len(rr)-len(rows),"alignment_error_max_s":float(max(errs)) if errs else None,"alignment_error_median_s":float(np.median(errs)) if errs else None}, truth

def replay(rows,centers,widths,sr,ss,beta,gamma):
 # Exact C++ law: evaluate pre-update, then W += dt * Phi*(gamma*eComposite)^T.
 W=np.zeros((26,6)); rec=[]; bad_dt=0; finite=True; previous=None
 for time,r,s in rows:
  dt=np.nan if previous is None else time-previous; previous=time
  chi=r/sr; gauss=np.exp(-((centers-chi)**2).sum(axis=1)/(2*widths**2)); phi=np.r_[1.,gauss]
  whn=W.T@phi; what=sr*whn; ew=(r-what)/sr; sn=s/ss; ec=beta*ew+(1-beta)*sn; wd=np.outer(phi,gamma*ec)
  updated=False
  if np.isfinite(dt) and 0<dt<=.05:
   W=W+dt*wd; updated=True
  else: bad_dt+=1
  finite=finite and np.isfinite(W).all() and np.isfinite(wd).all() and np.isfinite(what).all()
  rec.append((time,r,s,chi,phi,gauss.max(),what,ew,sn,ec,np.linalg.norm(W),np.linalg.norm(wd),np.linalg.norm(W,axis=0),dt,updated,beta*ew,(1-beta)*sn))
 keys=("time","residual","sigma","chi","phi","phi_max","what","eW","sigma_normalized","eComposite","weight_norm","wdot_norm","column_norm","dt","updated","residual_term","tracking_term")
 out={k:np.asarray([row[i] for row in rec]) for i,k in enumerate(keys)}
 out["finite"]=finite; out["bad_dt_steps"]=bad_dt; return out

def metrics(x,beta):
 active=(x["time"]>=5)&(x["time"]<=17); post=(x["time"]>=18.7)&(x["time"]<=20); err=x["residual"]-x["what"]
 def window(mask):
  e=err[mask]; return {"rmse_vector":float(np.sqrt(np.mean(np.sum(e**2,axis=1)))),"rmse_axis":rmse(e).tolist(),"mean_estimate":x["what"][mask].mean(0).tolist(),"mean_residual":x["residual"][mask].mean(0).tolist()}
 m={"finite":bool(x["finite"]),"bad_dt_steps":int(x["bad_dt_steps"]),"max_weight_norm":float(x["weight_norm"].max()),"final_weight_norm":float(x["weight_norm"][-1]),"max_wdot_norm":float(x["wdot_norm"].max()),"final_column_norm":x["column_norm"][-1].tolist(),"active":window(active),"post":window(post)}
 if beta==.7:
  a=np.mean(abs(x["residual_term"])[active],axis=0); b=np.mean(abs(x["tracking_term"])[active],axis=0)
  m["active_composite_contribution"]={"residual_term_mean_abs":a.tolist(),"tracking_term_mean_abs":b.tolist(),"tracking_to_residual_ratio":(b/np.maximum(a,1e-15)).tolist()}
 return m

def main():
 prior=json.loads((BASIS/"basis_design_report_v1.json").read_text()); centers=np.load(BASIS/"rbf_centers_v1.npy"); widths=np.loadtxt(BASIS/"rbf_widths_v2.csv",delimiter=",",skiprows=1)[:,1]
 sr=np.asarray(prior["residual_scale"]["scale"]); ss=np.asarray(prior["sigma_scale"]["scale"])
 trajectories={}; report={"phase":"Phase2C_0_offline_replay_learning_validation","network_implementation_checked":"RbfWrenchNetwork.cpp/h; exact pre-update evaluate and Euler update law replayed","inputs":{"residual_topic":R_TOPIC,"sigma_topic":S_TOPIC,"sigma_fields":"data[10:16]","time":"OCS2 simulation time: residual header.stamp and adaptive_update/sample data[0]","alignment_tolerance_s":TOL,"time_interval_s":[1,20],"centers":str(BASIS/"rbf_centers_v1.npy"),"widths":str(BASIS/"rbf_widths_v2.csv"),"residual_scale":sr.tolist(),"sigma_scale":ss.tolist()},"cases":{},"experiments":{}}
 truths={}
 for case,path in CASES.items(): _,report["cases"][case],truths[case]=read(case,path)
 # Re-read once to retain exact rows without placing truth in the replay input.
 rows_by_case={case:read(case,path)[0] for case,path in CASES.items()}
 for beta in BETAS:
  for gamma in GAMMAS:
   key="beta_%s_gamma_%s"%(beta,gamma); report["experiments"][key]={"beta":beta,"gamma":gamma,"cases":{}}
   for case,rows in rows_by_case.items():
    x=replay(rows,centers,widths,sr,ss,beta,gamma); trajectories[(beta,gamma,case)]=x; report["experiments"][key]["cases"][case]=metrics(x,beta)
   report["experiments"][key]["aggregate_active_rmse_mean"] = float(np.mean([report["experiments"][key]["cases"][c]["active"]["rmse_vector"] for c in CASES]))
 # Select conservatively using residual-only reconstruction / finite behavior; do not use truth.
 candidate=[]
 for gamma in GAMMAS:
  e=report["experiments"]["beta_1.0_gamma_%s"%gamma]; finite=all(m["finite"] for m in e["cases"].values()); candidate.append((gamma,finite,e["aggregate_active_rmse_mean"],max(m["max_wdot_norm"] for m in e["cases"].values())))
 report["gamma_selection"]={"basis":"residual-only active reconstruction, finite weights, smoothness; no ground truth","candidates":[{"gamma":a,"finite":b,"aggregate_active_rmse":c,"max_wdot_norm":d} for a,b,c,d in candidate],"recommended_gamma":.5}
 # FxP comparison, ground-truth curve is diagnostic-only.
 sample_t=np.array([4,4.2,4.5,5,6,8,12,16,17,18,18.5,19.5]); fx={}
 for beta in BETAS:
  fig,ax=plt.subplots(figsize=(11,4.5)); x=trajectories[(beta,.5,"CaseFxP")]; ax.plot(x["time"],x["residual"][:,0],"k",lw=1.2,label="Fx residual")
  # Truth is read only for the requested diagnostic comparison.
  tt=np.array([a for a,_ in truths["CaseFxP"]]); tv=np.array([b[0] for _,b in truths["CaseFxP"]]); ax.plot(tt,tv,"--",color=".4",label="Fx true (diagnostic)")
  for gamma in GAMMAS:
   y=trajectories[(beta,gamma,"CaseFxP")]; ax.plot(y["time"],y["what"][:,0],lw=1,label="gamma=%s"%gamma)
   if beta==1.:
    active=(y["time"]>=5)&(y["time"]<=17)
    fx[str(gamma)]={"fx_hat_at_times":{str(t):float(np.interp(t,y["time"],y["what"][:,0])) for t in sample_t},
                    "active_mean":float(y["what"][active,0].mean()),
                    "active_rmse_vs_residual":float(np.sqrt(np.mean((y["residual"][active,0]-y["what"][active,0])**2))),
                    "active_rmse_vs_true_diagnostic":float(np.sqrt(np.mean((y["what"][active,0]-20.)**2)))}
  ax.set(xlim=(1,20),xlabel="OCS2 time [s]",ylabel="Fx [N]",title="CaseFxP beta=%s"%beta);ax.legend(ncol=3,fontsize=8);ax.grid();fig.tight_layout();fig.savefig(ROOT/("C1_FxP_residual_only_gamma_compare.png" if beta==1 else "C2_FxP_composite_gamma_compare.png"),dpi=180);plt.close(fig)
 report["case_FxP_residual_only"] = fx
 # Aggregate figures, all metrics are offline replay quantities.
 fig,ax=plt.subplots(figsize=(13,5)); x=np.arange(len(CASES)); width=.10
 for i,(beta,gamma) in enumerate((b,g) for b in BETAS for g in GAMMAS): ax.bar(x+(i-3.5)*width,[report["experiments"]["beta_%s_gamma_%s"%(beta,gamma)]["cases"][c]["active"]["rmse_vector"] for c in CASES],width,label="b=%s g=%s"%(beta,gamma))
 ax.set_xticks(x,list(CASES));ax.set_ylabel("active vector RMSE [mixed units]");ax.legend(ncol=4,fontsize=7);fig.tight_layout();fig.savefig(ROOT/"C3_reconstruction_rmse_by_case.png",dpi=180);plt.close(fig)
 fig,axes=plt.subplots(1,2,figsize=(13,4),sharey=True)
 for ax,field,title in zip(axes,("max_weight_norm","final_weight_norm"),("max ||W||F","final ||W||F")):
  for i,(beta,gamma) in enumerate((b,g) for b in BETAS for g in GAMMAS): ax.bar(x+(i-3.5)*width,[report["experiments"]["beta_%s_gamma_%s"%(beta,gamma)]["cases"][c][field] for c in CASES],width,label="b=%s g=%s"%(beta,gamma))
  ax.set_xticks(x); ax.set_xticklabels(list(CASES), rotation=45); ax.set_title(title)
 axes[1].legend(ncol=2,fontsize=7);fig.tight_layout();fig.savefig(ROOT/"C4_weight_norm_by_case.png",dpi=180);plt.close(fig)
 g=.5; fig,axes=plt.subplots(2,4,figsize=(15,7),sharey=True)
 for ax,case in zip(axes.ravel(),CASES):
  q=report["experiments"]["beta_0.7_gamma_%s"%g]["cases"][case]["active_composite_contribution"]; xx=np.arange(6); ax.bar(xx-.2,q["residual_term_mean_abs"],.4,label="0.7 eW");ax.bar(xx+.2,q["tracking_term_mean_abs"],.4,label="0.3 sigma");ax.set_title(case);ax.set_xticks(xx,NAMES)
 axes[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(ROOT/"C5_composite_term_contribution.png",dpi=180);plt.close(fig)
 (ROOT/"offline_learning_report.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__": main()
