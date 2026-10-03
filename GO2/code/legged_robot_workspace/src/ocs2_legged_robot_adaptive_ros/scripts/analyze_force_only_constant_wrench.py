#!/usr/bin/env python3
"""Offline plots/statistics for the force-only ConstantWrench6D run."""
import json
import sys
import numpy as np
import rosbag
import matplotlib.pyplot as plt

GAMMA_FX = 3.0
T0 = 4.0

def interp(t, y, x):
    return float(np.interp(x, t, y))

def mask(t, lo, hi, include_hi=True):
    return (t >= lo) & ((t <= hi) if include_hi else (t < hi))

def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))

def slope(t, y):
    return float(np.polyfit(t, y, 1)[0]) if len(t) >= 2 else float('nan')

def main(bag_path, output_dir, runtime=20.0, active_end=18.0, prefix='F', gamma_fx=3.0):
    rows = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=['/adaptive_update/sample']):
            if len(msg.data) == 16:
                rows.append(np.asarray(msg.data, dtype=float))
    if len(rows) < 10:
        raise RuntimeError('missing or insufficient /adaptive_update/sample (expected 16-field 6D samples)')
    a = np.vstack(rows)
    a = a[np.argsort(a[:, 0])]
    t, fx_hat, sx = a[:, 0], a[:, 4], a[:, 10]
    # A strictly increasing sample-time sequence is required for the integral.
    keep = np.r_[True, np.diff(t) > 1e-9]
    t, fx_hat, sx = t[keep], fx_hat[keep], sx[keep]
    # The runner allows a short bag-flush grace period after MPC time 20 s.
    # The prescribed experiment/plots/statistics are strictly the 0--20 s run.
    within_runtime = t <= runtime + 1e-9
    t, fx_hat, sx = t[within_runtime], fx_hat[within_runtime], sx[within_runtime]
    fx_true = np.where((t >= T0) & (t <= active_end), 20.0, 0.0)
    mactive = mask(t, T0, active_end)
    last_window_seconds = 20.0 if runtime > 20.0 else 5.0
    mlast = mask(t, active_end - last_window_seconds, active_end)
    ti, si = t[t >= T0], sx[t >= T0]
    integral = np.zeros_like(ti)
    integral[1:] = np.cumsum(0.5 * (si[1:] + si[:-1]) * np.diff(ti))
    delta_hat = fx_hat[t >= T0] - interp(t, fx_hat, T0)
    predicted_delta = gamma_fx * integral
    points_at = ((4, 20, 40, 60, 80, 100, 120, 124, 130) if runtime > 20.0
                 else ((4, 6, 8, 10, 12, 14, 16, 18, 20) if gamma_fx != 3.0
                       else (4, 8, 12, 16, 18, 20)))
    points = {str(q): interp(t, fx_hat, q) for q in points_at}
    active_hat = fx_hat[mactive]
    final18 = interp(t, fx_hat, active_end)
    sigma_active = sx[mactive]
    sigma_windows = {}
    if runtime > 20.0:
        for lo, hi in ((4.0, 40.0), (40.0, 80.0), (80.0, 124.0)):
            mw = mask(t, lo, hi)
            sw = sx[mw]
            sigma_windows[f'{int(lo)}_{int(hi)}'] = {
                'mean': float(np.mean(sw)), 'mean_abs': float(np.mean(np.abs(sw))),
                'rms': rms(sw), 'integral': float(np.trapz(sw, t[mw]))}
    result = {
        'samples': int(len(t)), 'time_min': float(t.min()), 'time_max': float(t.max()),
        'fx_hat_at_s': points,
        'active_mean_fx_hat': float(np.mean(active_hat)),
        'runtime': runtime, 'active_end': active_end,
        'gamma_fx': gamma_fx,
        'final_active_fx_hat': final18, 'active_max_fx_hat': float(np.max(active_hat)),
        'overshoot_present': bool(np.any(active_hat > 20.0)),
        'max_overshoot': float(max(0.0, np.max(active_hat) - 20.0)),
        'final_active_absolute_error': float(abs(final18 - 20.0)),
        'final_active_relative_error_percent': float(abs(final18 - 20.0) / 20.0 * 100.0),
        'last_window_seconds': last_window_seconds,
        'last_active_mean': float(np.mean(fx_hat[mlast])),
        'last_active_std': float(np.std(fx_hat[mlast])),
        'last_active_slope_N_per_s': slope(t[mlast], fx_hat[mlast]),
        'sigma_x_active_mean': float(np.mean(sigma_active)),
        'sigma_x_active_mean_abs': float(np.mean(np.abs(sigma_active))),
        'sigma_x_active_rms': rms(sigma_active),
        'sigma_x_active_integral': float(np.trapz(sigma_active, t[mactive])),
        'sigma_x_positive_ratio': float(np.mean(sigma_active > 0.0)),
        'sigma_x_negative_ratio': float(np.mean(sigma_active < 0.0)),
        'sigma_x_windows': sigma_windows,
        'integral_comparison_final_actual_delta': float(delta_hat[-1]),
        'integral_comparison_final_predicted_delta': float(predicted_delta[-1]),
        'integral_comparison_final_difference': float(delta_hat[-1] - predicted_delta[-1]),
        'integral_comparison_rms_difference': rms(delta_hat - predicted_delta),
        'integral_comparison_max_abs_difference': float(np.max(np.abs(delta_hat - predicted_delta))),
    }
    # F1
    fig, ax = plt.subplots(figsize=(9, 4.8)); ax.axvspan(T0, active_end, color='0.88', label='disturbance active')
    ax.plot(t, fx_true, '--', lw=2, color='k', label='true $F_x$')
    ax.plot(t, fx_hat, lw=2, color='#0072BD', label='estimated $F_x$')
    ax.set(xlim=(0,runtime), xlabel='MPC time [s]', ylabel='$F_x$ [N]'); ax.grid(True, alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(output_dir + f'/{prefix}1_Fx_hat_vs_true' + ('_long' if prefix == 'L' else '') + '.png', dpi=180); plt.close(fig)
    # F2
    fig, ax = plt.subplots(figsize=(9, 4.8)); ax.plot(t, sx, lw=1.3, color='#D95319', label='$\\sigma_x$')
    ax.axhline(0, color='k', lw=1); ax.set(xlim=(0,runtime), xlabel='MPC time [s]', ylabel='$\\sigma_x$'); ax.grid(True, alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(output_dir + f'/{prefix}2_sigma_x' + ('_long' if prefix == 'L' else '') + '.png', dpi=180); plt.close(fig)
    # F3
    fig, ax = plt.subplots(figsize=(9, 4.8)); ax.axvspan(T0, active_end, color='0.88', label='disturbance active')
    ax.plot(ti, delta_hat, lw=2, label='$\\hat F_x(t)-\\hat F_x(4)$')
    ax.plot(ti, predicted_delta, '--', lw=2, label=f'${gamma_fx:g}\\int_4^t \\sigma_x\\,d\\tau$')
    ax.set(xlim=(0,runtime), xlabel='MPC time [s]', ylabel='force-estimate change [N]'); ax.grid(True, alpha=.3); ax.legend(); fig.tight_layout()
    fig.savefig(output_dir + f'/{prefix}3_Fx_hat_update_integral' + ('_long' if prefix == 'L' else '') + '.png', dpi=180); plt.close(fig)
    with open(output_dir + '/force_only_analysis.json', 'w') as f: json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    if len(sys.argv) not in (3, 6, 7):
        raise SystemExit('usage: analyze_force_only_constant_wrench.py BAG OUT [RUNTIME ACTIVE_END PREFIX [GAMMA_FX]]')
    if len(sys.argv) == 3:
        main(sys.argv[1], sys.argv[2])
    else:
        main(sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5],
             float(sys.argv[6]) if len(sys.argv) == 7 else 3.0)
