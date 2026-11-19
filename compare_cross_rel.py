#!/usr/bin/env python3
"""Cross position in the drone's base frame: vSLAM (/cross_rel/vslam) against OptiTrack (/cross_rel/gt).

Two steps, because rosbag2 types live in the container and matplotlib on the host:
  1) inside the container:  python3 compare_cross_rel.py extract bags/<bag> bags/<bag>.npz
  2) anywhere with numpy/matplotlib:  python3 compare_cross_rel.py analyze bags/<bag>.npz out.png [t0 t1]

Both topics are the vector from base_link to the cross in base_link axes (x forward, y left,
z up): d455_mapping cross_rel_base (map) and vanguard cross_rel_gt (OptiTrack). The OptiTrack
vector is interpolated to the vSLAM stamps; a lag between the two stamp clocks is searched
(+-0.3 s) and reported, the statistics are at the best lag. [t0 t1] = seconds since the first
vSLAM sample to restrict the comparison (e.g. only while hovering in front of the cross).
"""
import glob
import os
import sys

import numpy as np


def extract(bag, out):
    import sqlite3

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    c = sqlite3.connect(f'file:{glob.glob(os.path.join(bag, "*.db3"))[0]}?mode=ro', uri=True)
    names = {i: n for i, n, _ in c.execute('select id, name, type from topics')}
    types = {n: get_message(t) for _, n, t in c.execute('select id, name, type from topics')}
    rows = {'/cross_rel/vslam': [], '/cross_rel/gt': []}
    for tid, t, data in c.execute('select topic_id, timestamp, data from messages order by timestamp'):
        if names[tid] in rows:
            m = deserialize_message(bytes(data), types[names[tid]])
            rows[names[tid]].append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                     m.vector.x, m.vector.y, m.vector.z])
    np.savez(out, vslam=np.array(rows['/cross_rel/vslam']), gt=np.array(rows['/cross_rel/gt']))
    print(out, {k: len(v) for k, v in rows.items()})


def match(vs, gt, lag):
    """OptiTrack vector at the vSLAM stamps (+lag): (mask, gt_at_vs)."""
    t = vs[:, 0] + lag
    ok = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    g = np.stack([np.interp(t[ok], gt[:, 0], gt[:, i]) for i in (1, 2, 3)], 1)
    return ok, g


def analyze(npz, png, t0=None, t1=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    D = np.load(npz)
    vs, gt = D['vslam'], D['gt']
    if len(vs) < 5 or len(gt) < 5:
        sys.exit(f'too few samples: vslam {len(vs)}, gt {len(gt)}')
    T0 = vs[0, 0]
    sel = np.ones(len(vs), bool)
    if t0 is not None:
        sel &= vs[:, 0] - T0 >= t0
    if t1 is not None:
        sel &= vs[:, 0] - T0 <= t1
    vs = vs[sel]

    def rms_at(lag):
        ok, g = match(vs, gt, lag)
        return np.sqrt(np.mean(np.sum((vs[ok, 1:] - g) ** 2, 1))) if ok.sum() > 5 else np.inf
    lags = np.arange(-0.3, 0.3001, 0.01)
    lag = min(lags, key=rms_at)
    ok, g = match(vs, gt, lag)
    v, tt = vs[ok, 1:], vs[ok, 0] - T0
    e = v - g
    dist_gt = np.linalg.norm(g, axis=1)
    de = np.linalg.norm(v, axis=1) - dist_gt

    def row(name, x):
        return (f'  {name:<10} mean {x.mean() * 100:+6.1f}  std {x.std() * 100:5.1f}  '
                f'rms {np.sqrt((x ** 2).mean()) * 100:5.1f}  max |.| {np.abs(x).max() * 100:5.1f} cm')
    print(f'{npz}: {ok.sum()} vSLAM samples compared, distance to the cross {dist_gt.min():.2f}-{dist_gt.max():.2f} m')
    print(f'  best lag {lag * 1e3:+.0f} ms (rms {rms_at(lag) * 100:.1f} cm; at 0 ms {rms_at(0.0) * 100:.1f} cm)')
    print('error = vSLAM - OptiTrack, base_link axes (x forward, y left, z up):')
    for i, n in enumerate('xyz'):
        print(row(n, e[:, i]))
    print(row('3D', np.linalg.norm(e, axis=1)))
    print(row('distance', de))
    bins = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 99)]
    for a, b in bins:
        m = (dist_gt >= a) & (dist_gt < b)
        if m.sum() > 5:
            print(f'  distance {a}-{b if b < 99 else "+"} m: n {m.sum():5d}  3D rms {np.sqrt((np.sum(e[m] ** 2, 1)).mean()) * 100:5.1f} cm  '
                  f'bias {np.round(e[m].mean(0) * 100, 1)} cm')

    fig, ax = plt.subplots(3, 2, figsize=(13, 9))
    for i, n in enumerate('xyz'):
        ax[i, 0].plot(tt, g[:, i], label='OptiTrack')
        ax[i, 0].plot(tt, v[:, i], lw=1, label='vSLAM')
        ax[i, 0].set_ylabel(f'{n} [m]')
        ax[i, 0].grid(alpha=.3)
        ax[i, 1].plot(tt, e[:, i] * 100)
        ax[i, 1].axhline(0, color='k', lw=.5)
        ax[i, 1].set_ylabel(f'vSLAM - OptiTrack {n} [cm]')
        ax[i, 1].grid(alpha=.3)
    ax[0, 0].legend()
    ax[0, 0].set_title('cross in base_link axes')
    ax[0, 1].set_title(f'error, lag {lag * 1e3:+.0f} ms, 3D rms {np.sqrt((np.sum(e ** 2, 1)).mean()) * 100:.1f} cm')
    ax[2, 0].set_xlabel('time [s]')
    ax[2, 1].set_xlabel('time [s]')
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    fig2, a2 = plt.subplots(1, 2, figsize=(11, 4))
    a2[0].scatter(dist_gt, np.linalg.norm(e, axis=1) * 100, s=4)
    a2[0].set_xlabel('true distance to the cross [m]')
    a2[0].set_ylabel('3D error [cm]')
    a2[0].grid(alpha=.3)
    a2[1].scatter(dist_gt, de * 100, s=4)
    a2[1].axhline(0, color='k', lw=.5)
    a2[1].set_xlabel('true distance to the cross [m]')
    a2[1].set_ylabel('distance error [cm]')
    a2[1].grid(alpha=.3)
    fig2.tight_layout()
    fig2.savefig(png.replace('.png', '_distance.png'), dpi=110)


if __name__ == '__main__':
    if len(sys.argv) >= 4 and sys.argv[1] == 'extract':
        extract(sys.argv[2], sys.argv[3])
    elif len(sys.argv) >= 4 and sys.argv[1] == 'analyze':
        analyze(sys.argv[2], sys.argv[3], *(float(a) for a in sys.argv[4:6]))
    else:
        sys.exit(__doc__)
