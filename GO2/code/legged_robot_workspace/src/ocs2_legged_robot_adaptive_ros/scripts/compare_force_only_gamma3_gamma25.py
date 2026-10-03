#!/usr/bin/env python3
"""Direct offline comparison of the two prescribed 20 s force-only bags."""
import sys
import numpy as np
import rosbag
import matplotlib.pyplot as plt

def read(path):
    rows=[]
    with rosbag.Bag(path) as bag:
        for _,m,_ in bag.read_messages(topics=['/adaptive_update/sample']):
            if len(m.data)==16: rows.append(m.data)
    a=np.asarray(rows,float); a=a[np.argsort(a[:,0])]
    keep=np.r_[True,np.diff(a[:,0])>1e-9] & (a[:,0]<=20.0+1e-9)
    return a[keep,0],a[keep,4],a[keep,10]

def main(bag3,bag25,out):
    t3,f3,s3=read(bag3); t25,f25,s25=read(bag25)
    fig,ax=plt.subplots(figsize=(9,4.8)); ax.axvspan(4,18,color='0.88',label='disturbance active')
    tt=np.linspace(0,20,1001); ax.plot(tt,np.where((tt>=4)&(tt<=18),20,0),'--',color='k',lw=2,label='true $F_x$')
    ax.plot(t3,f3,lw=2,label='$\\hat F_x$, $\\Gamma_{Fx}=3$'); ax.plot(t25,f25,lw=2,label='$\\hat F_x$, $\\Gamma_{Fx}=25$')
    ax.set(xlim=(0,20),xlabel='MPC time [s]',ylabel='$F_x$ [N]');ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(out+'/G_FORCE_1_Fx_hat_gamma3_vs_gamma25_vs_true.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4.8));ax.plot(t3,s3,lw=1.3,label='$\\sigma_x$, $\\Gamma_{Fx}=3$');ax.plot(t25,s25,lw=1.3,label='$\\sigma_x$, $\\Gamma_{Fx}=25$');ax.axhline(0,color='k',lw=1)
    ax.set(xlim=(0,20),xlabel='MPC time [s]',ylabel='$\\sigma_x$');ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(out+'/G_FORCE_2_sigma_x_gamma3_vs_gamma25.png',dpi=180);plt.close(fig)
    active=t25>=4; ti,si=t25[active],s25[active]
    integ=np.zeros_like(ti); integ[1:]=np.cumsum(.5*(si[1:]+si[:-1])*np.diff(ti))
    base=float(np.interp(4,t25,f25)); delta=f25[active]-base
    fig,ax=plt.subplots(figsize=(9,4.8));ax.axvspan(4,18,color='0.88',label='disturbance active')
    ax.plot(ti,delta,lw=2,label='$\\hat F_x(t)-\\hat F_x(4)$');ax.plot(ti,25*integ,'--',lw=2,label='$25\\int_4^t\\sigma_x\\,d\\tau$')
    ax.set(xlim=(0,20),xlabel='MPC time [s]',ylabel='force-estimate change [N]');ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(out+'/G_FORCE_3_gamma25_update_integral.png',dpi=180);plt.close(fig)

if __name__=='__main__':
    if len(sys.argv)!=4: raise SystemExit('usage: compare_force_only_gamma3_gamma25.py GAMMA3.BAG GAMMA25.BAG OUTDIR')
    main(*sys.argv[1:])
