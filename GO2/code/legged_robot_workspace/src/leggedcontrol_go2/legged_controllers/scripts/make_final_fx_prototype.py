#!/usr/bin/env python3
"""Create advisor-plot prototypes solely from validated Phase 1.5 X-force CSVs."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[6]
PURE = ROOT / "experi/results/phase1_5_xforce/20261004T174833.104926Z_10540ca3/advisor_force__flat_velocity_fx_constant__pure_mpc_standard__r001"
RBF = ROOT / "experi/results/phase1_5_xforce/20261004T175449.478389Z_54ad7897/advisor_force__flat_velocity_fx_constant__v11_equivalent_wrench_standard__r001"
OUT = ROOT / "experi/results/final_fx_advisor/prototype"
OUT.mkdir(parents=True, exist_ok=True)
PERIOD = 0.5  # trot gait cycle: two diagonal contact phases at 2 Hz

class Table:
    def __init__(self, data): self.data=data
    def __getattr__(self, key): return self.data[key]
    def __getitem__(self, key): return self.data[key]
def read_csv(path): return Table(np.genfromtxt(path, delimiter=',', names=True, dtype=float, encoding='utf-8'))
def tframe(df): return np.asarray(df.wall_elapsed, dtype=float)
def avg(t, y):
    # causal one-period rectangular average on a common 50-Hz display grid.
    grid=np.arange(0.,20.,.02); yy=np.interp(grid,t,y); n=round(PERIOD/.02)
    return grid, np.convolve(yy,np.ones(n)/n,mode="same")
def truth(state, wrench, td):
    # Exact deployed RBF convention: d=Y*pi is BODY / overall-CoM / left-hand.
    # physical world force application is mapped F_B=R_WB^T F_W; moment is
    # shifted base->CoM via the model contract then left-hand d=-[F_B;tau_B].
    s=state; ts=tframe(s); tw=tframe(wrench)
    cols=['fx','fy','fz','tx','ty','tz']; w=np.column_stack([np.interp(td,tw,np.asarray(wrench[c],dtype=float)) for c in cols])
    yaw=np.interp(td,ts,s.observation_base_yaw); pitch=np.interp(td,ts,s.observation_base_pitch); roll=np.interp(td,ts,s.observation_base_roll)
    cy,sy=np.cos(yaw),np.sin(yaw); cp,sp=np.cos(pitch),np.sin(pitch); cr,sr=np.cos(roll),np.sin(roll)
    R=np.empty((len(td),3,3)); R[:,0,:]=np.c_[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr]; R[:,1,:]=np.c_[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr]; R[:,2,:]=np.c_[-sp,cp*sr,cp*cr]
    fb=np.einsum('nij,nj->ni',np.transpose(R,(0,2,1)),w[:,:3])
    # nominal Go2 overall-CoM -> base-origin BODY vector, audited contract.
    c=np.array([0.000229,-0.003089,0.027360])
    tau=np.einsum('nij,nj->ni',np.transpose(R,(0,2,1)),w[:,3:])+np.cross(c,fb)
    return -np.c_[fb,tau]
def setup(ax,title,ylabel):
    ax.set(title=title,xlabel="Time [s]",ylabel=ylabel,xlim=(0,20)); ax.axvline(6,color='k',ls='--',lw=1); ax.text(6.15,ax.get_ylim()[1]*.9,"World +X 20 N",fontsize=9); ax.grid(alpha=.25)
pure=read_csv(PURE/'state.csv'); rstate=read_csv(RBF/'state.csv'); diag=read_csv(RBF/'v11_equivalent_wrench.csv'); wrench=read_csv(RBF/'wrench.csv')
tp,tr=tframe(pure),tframe(rstate); vp=np.asarray(pure.base_vx,dtype=float); vr=np.asarray(rstate.base_vx,dtype=float)
ref=np.where(tr<2,0,np.where(tr<4,.45*(tr-2)/2,.45)); gp,ap=avg(tp,vp); gr,ar=avg(tr,vr); rr=np.interp(gp,tr,ref)
fig,ax=plt.subplots(figsize=(10,5)); ax.plot(tr,ref,'k--',lw=2,label='Reference'); ax.plot(tp,vp,color='tab:blue',alpha=.25,lw=.7,label='Pure MPC raw'); ax.plot(tr,vr,color='tab:orange',alpha=.25,lw=.7,label='Current RBF raw'); ax.plot(gp,ap,color='tab:blue',lw=2.4,label='Pure MPC gait-cycle avg'); ax.plot(gr,ar,color='tab:orange',lw=2.4,label='Current RBF gait-cycle avg'); setup(ax,'World +X disturbance: forward-velocity tracking','Forward velocity $v_x$ [m/s]'); ax.legend(ncol=2,fontsize=9); fig.tight_layout(); fig.savefig(OUT/'prototype_vx_tracking.png',dpi=240); plt.close(fig)
fig,ax=plt.subplots(figsize=(10,5)); ax.plot(tp,vp-np.interp(tp,tr,ref),color='tab:blue',alpha=.25,lw=.7,label='Pure MPC raw'); ax.plot(tr,vr-ref,color='tab:orange',alpha=.25,lw=.7,label='Current RBF raw'); ax.plot(gp,ap-rr,color='tab:blue',lw=2.4,label='Pure MPC gait-cycle avg'); ax.plot(gr,ar-np.interp(gr,tr,ref),color='tab:orange',lw=2.4,label='Current RBF gait-cycle avg'); ax.axhline(0,color='k',lw=.8); setup(ax,'World +X disturbance: forward-velocity tracking error','$v_x-v_{x,ref}$ [m/s]'); ax.legend(ncol=2,fontsize=9); fig.tight_layout(); fig.savefig(OUT/'prototype_vx_error.png',dpi=240); plt.close(fig)
td=tframe(diag); hat=np.column_stack([np.asarray(diag['active_equivalent_wrench_body_'+a],dtype=float) for a in ['x','y','z','rx','ry','rz']]); err=hat-truth(rstate,wrench,td)
fig,ax=plt.subplots(figsize=(10,4)); ax.plot(td,err[:,0],color='tab:orange',lw=1.3,label='Current RBF $e_{F_x}$'); ax.axhline(0,color='k',ls='--'); setup(ax,'RBF Fx prediction error (active estimate)','$e_{F_x}$ [N]'); ax.legend(); fig.tight_layout(); fig.savefig(OUT/'prototype_RBF_Fx_prediction_error.png',dpi=240); plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4)); ax.plot(td,np.linalg.norm(err[:,:3],axis=1),color='tab:orange',lw=1.3,label=r'$||e_F||_2$'); setup(ax,'RBF force prediction-error norm','$||e_F||_2$ [N]'); ax.legend(); fig.tight_layout(); fig.savefig(OUT/'prototype_RBF_force_error_norm.png',dpi=240); plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(11,8),sharex=True); labels=['Fx [N]','Fy [N]','Fz [N]','Tx [Nm]','Ty [Nm]','Tz [Nm]'];
for i,ax in enumerate(axs.flat): ax.plot(td,err[:,i],color='tab:orange',lw=1); ax.axhline(0,color='k',ls='--',lw=.7); ax.axvline(6,color='k',ls=':',lw=.8); ax.set_ylabel(labels[i]); ax.grid(alpha=.2)
for ax in axs[-1]: ax.set_xlabel('Time [s]')
fig.suptitle('RBF six-dimensional prediction error (active estimate minus trajectory truth)'); fig.tight_layout(); fig.savefig(OUT/'prototype_RBF_six_dim_prediction_error.png',dpi=240); plt.close(fig)
print(OUT)
