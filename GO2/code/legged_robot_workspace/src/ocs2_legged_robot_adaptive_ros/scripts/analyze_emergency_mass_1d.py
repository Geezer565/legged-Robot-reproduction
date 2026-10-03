#!/usr/bin/env python3
"""Paired 20 s analysis for N/M0/M1 PayloadMass1D acquisition bags."""
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

PAYLOAD = "/adaptive_update/payload_mass_sample"
OBS = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
SIGMA = "/aclf_experiment/executed_sigma_norm"
TRUE_PAYLOAD = 5.213485

def desired(t, knots, states):
    if t <= knots[0]: return np.asarray(states[0])
    if t >= knots[-1]: return np.asarray(states[-1])
    i = np.searchsorted(knots, t); a = (knots[i]-t)/(knots[i]-knots[i-1])
    return a*np.asarray(states[i-1]) + (1-a)*np.asarray(states[i])

def load(path):
    p, obs, sig, target = [], [], [], None
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[PAYLOAD, OBS, TARGET, SIGMA]):
            if topic == PAYLOAD:
                if len(msg.data) != 21: raise RuntimeError("bad payload schema")
                p.append(np.asarray(msg.data, float))
            elif topic == OBS:
                obs.append((float(msg.time), stamp.to_sec(), np.asarray(msg.state.value, float)))
            elif topic == TARGET:
                target = (np.asarray(msg.timeTrajectory, float), [x.value for x in msg.stateTrajectory])
            else: sig.append((stamp.to_sec(), float(msg.data)))
    if not p or not obs or target is None or not sig: raise RuntimeError("required recorded topic missing")
    p=np.vstack(p); obs.sort(); ot=np.array([x[0] for x in obs]); ob=np.array([x[1] for x in obs]); os=np.vstack([x[2] for x in obs])
    de=np.vstack([desired(t,*target) for t in ot]); pos=de[:,6:9]-os[:,6:9]
    sig=np.asarray(sig); st=np.interp(sig[:,0],ob,ot)
    return {"mass_t":p[:,0], "mass":p[:,5], "t":ot, "zerr":pos[:,2], "poserr":np.linalg.norm(pos,axis=1), "sigma_t":st, "sigma":sig[:,1]}

def metric(t,x):
    m=(t>=0)&(t<=20); q=x[m]
    return {"samples":int(len(q)), "mean":float(q.mean()), "rms":float(np.sqrt(np.mean(q*q)))}

def plot(items, ylabel, out, hlines=()):
    plt.figure(figsize=(9,4.5))
    for label,t,x in items: plt.plot(t,x,label=label,linewidth=1.3)
    for y,label in hlines: plt.axhline(y,color="k",ls="--",lw=.9,label=label)
    plt.xlim(0,20);plt.xlabel("MPC time [s]");plt.ylabel(ylabel);plt.grid(True);plt.legend();plt.tight_layout();plt.savefig(out,dpi=180);plt.close()

def main():
    if len(sys.argv)!=3: raise SystemExit("usage: analyze_emergency_mass_1d.py ROOT OUTPUT")
    root,out=Path(sys.argv[1]),Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
    d={k:load(root/(k+"_final")/"payload_mass_smoke.bag") for k in ("N","M0","M1")}
    m=d["M1"]; last=(m["mass_t"]>=15)&(m["mass_t"]<=20); mt=m["mass_t"][last]; mv=m["mass"][last]
    slope=float(np.polyfit(mt,mv,1)[0])
    report={"protocol":{"runtime_s":20,"true_payload_N_kg":0.0,"true_payload_M0_M1_kg":TRUE_PAYLOAD,"external_wrench":False,"com_offset":False,"inertia_mismatch":False},
            "M1":{"final_mass_hat_kg":float(m["mass"][-1]),"last5s_mean_kg":float(mv.mean()),"last5s_std_kg":float(mv.std(ddof=0)),"last5s_slope_kg_per_s":slope,"relative_error_final":float(abs(m["mass"][-1]-TRUE_PAYLOAD)/TRUE_PAYLOAD)},
            "M0":{},"comparisons":{}}
    for k in ("M0","M1"):
        report[k].update({"z_error":metric(d[k]["t"],d[k]["zerr"]),"position_error":metric(d[k]["t"],d[k]["poserr"]),"sigma_norm":metric(d[k]["sigma_t"],d[k]["sigma"])})
    report["comparisons"]={"M0_vs_M1_z_RMSE_m":{"M0":report["M0"]["z_error"]["rms"],"M1":report["M1"]["z_error"]["rms"]},"M0_vs_M1_position_RMSE_m":{"M0":report["M0"]["position_error"]["rms"],"M1":report["M1"]["position_error"]["rms"]},"M0_vs_M1_sigma_mean":{"M0":report["M0"]["sigma_norm"]["mean"],"M1":report["M1"]["sigma_norm"]["mean"]}}
    plot([("N mass_hat",d["N"]["mass_t"],d["N"]["mass"]),("M1 mass_hat",m["mass_t"],m["mass"])],"payload mass estimate [kg]",out/"N_vs_M1_mass_hat.png",[(0,"N true"),(TRUE_PAYLOAD,"M1 true")])
    plot([("M1 mass_hat",m["mass_t"],m["mass"])],"payload mass estimate [kg]",out/"M1_mass_hat.png",[(TRUE_PAYLOAD,"true")])
    plot([("M0",d["M0"]["t"],d["M0"]["zerr"]),("M1",m["t"],m["zerr"])],"z error: desired - actual [m]",out/"M0_vs_M1_z_error.png")
    plot([("M0",d["M0"]["t"],d["M0"]["poserr"]),("M1",m["t"],m["poserr"])],"position error norm [m]",out/"M0_vs_M1_position_error.png")
    plot([("M0",d["M0"]["sigma_t"],d["M0"]["sigma"]),("M1",m["sigma_t"],m["sigma"])],"sigma norm",out/"M0_vs_M1_sigma_norm.png")
    (out/"mass_1d_report.json").write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
if __name__=="__main__": main()
