"""Build a Nav2 occupancy map (pgm + yaml) from the LiDAR scans and optimized
poses stored in an RTAB-Map database, plus a top-view preview PNG.

Usage: python -I rtabmap_db_to_grid.py <map.db> <out_dir> <name> [resolution]
"""
import os
import sqlite3
import struct
import sys
import zlib

import numpy as np
from PIL import Image


def uncompress(blob):
    rows, cols, typ = struct.unpack('<iii', blob[-12:])
    raw = zlib.decompress(blob[:-12])
    depth = typ & 7
    channels = (typ >> 3) + 1
    dtype = {0: np.uint8, 1: np.int8, 2: np.uint16, 3: np.int16, 4: np.int32, 5: np.float32, 6: np.float64}[depth]
    return np.frombuffer(raw, dtype=dtype).reshape(rows, cols * channels), channels


def optimize_graph(con):
    """2D pose-graph optimization (x, y, yaw) for databases without saved
    optimized poses (e.g. recovered after a crash)."""
    from scipy.optimize import least_squares

    def mat(b):
        m = np.eye(4)
        m[:3] = np.frombuffer(b, np.float32).reshape(3, 4)
        return m

    odom = {i: mat(p) for i, p in con.execute('select id, pose from Node where weight >= 0')}
    ids = np.array(sorted(odom))
    idx = {i: k for k, i in enumerate(ids)}
    links = []
    for a, b, t, tf, inf in con.execute('select from_id, to_id, type, transform, information_matrix from Link where type in (0,1,2,3,4)'):
        if a in idx and b in idx:
            m = mat(tf)
            o = np.linalg.inv(odom[a]) @ odom[b]
            dyaw = np.angle(np.exp(1j * (np.arctan2(m[1, 0], m[0, 0]) - np.arctan2(o[1, 0], o[0, 0]))))
            # Odometry is locally good: drop closures that disagree with it a lot (bad ICP on in-place turns).
            if t != 0 and (abs(dyaw) > np.radians(5) or np.hypot(*(m[:2, 3] - o[:2, 3])) > 0.3):
                continue
            # Stored ICP covariances are far too optimistic (mm / 0.04 deg), so use fixed sigmas.
            w = (1 / 0.03, 1 / 0.03, 1 / np.radians(2)) if t == 0 else (1 / 0.1, 1 / 0.1, 1 / np.radians(3))
            links.append((idx[a], idx[b], m[0, 3], m[1, 3], np.arctan2(m[1, 0], m[0, 0]), *w))
    links = np.array(links)
    a, b = links[:, 0].astype(int), links[:, 1].astype(int)
    x0 = np.array([[odom[i][0, 3], odom[i][1, 3], np.arctan2(odom[i][1, 0], odom[i][0, 0])] for i in ids])

    def residual(v):
        p = v.reshape(-1, 3)
        p = np.vstack([x0[:1], p[1:]])
        c, s = np.cos(p[a, 2]), np.sin(p[a, 2])
        dx, dy = p[b, 0] - p[a, 0], p[b, 1] - p[a, 1]
        ex = c * dx + s * dy - links[:, 2]
        ey = -s * dx + c * dy - links[:, 3]
        et = np.angle(np.exp(1j * (p[b, 2] - p[a, 2] - links[:, 4])))
        # Odometry yaw is absolute (EKF fuses IMU yaw): keep it as a weak prior so
        # biased ICP yaw cannot bend legs that have no closure between them.
        prior = np.angle(np.exp(1j * (p[:, 2] - x0[:, 2]))) / np.radians(3)
        return np.concatenate([ex * links[:, 5], ey * links[:, 6], et * links[:, 7], prior])

    r0 = residual(x0.ravel())
    sol = least_squares(residual, x0.ravel(), loss='cauchy', f_scale=2.0)
    p = sol.x.reshape(-1, 3)
    p[0] = x0[0]
    print(f'{len(ids)} nodes, {len(links)} links, cost {r0 @ r0 / 2:.0f} -> {sol.cost:.0f}, '
          f'max shift {np.hypot(*(p[:, :2] - x0[:, :2]).T).max():.2f} m')
    poses = np.zeros((len(ids), 3, 4))
    for k, (x, y, th) in enumerate(p):
        poses[k] = [[np.cos(th), -np.sin(th), 0, x], [np.sin(th), np.cos(th), 0, y], [0, 0, 1, odom[ids[k]][2, 3]]]
    return ids, poses


def main():
    db, out_dir, name = sys.argv[1:4]
    res = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
    max_range = 8.0
    os.makedirs(out_dir, exist_ok=True)
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)

    ids_blob, poses_blob = con.execute('select opt_ids, opt_poses from Admin').fetchone()
    if ids_blob is None:
        ids, poses = optimize_graph(con)
    else:
        ids = uncompress(ids_blob)[0].reshape(-1)
        poses = uncompress(poses_blob)[0].reshape(-1, 3, 4).astype(np.float64)
    print(f'{len(ids)} optimized poses')

    origins, ends = [], []
    for node_id, pose in zip(ids, poses):
        row = con.execute('select scan, scan_info from Data where id=?', (int(node_id),)).fetchone()
        if not row or row[0] is None:
            continue
        scan, channels = uncompress(row[0])
        pts = scan.reshape(-1, channels)[:, :2].astype(np.float64)
        info = np.frombuffer(row[1], dtype=np.float32)
        local = info[-12:].reshape(3, 4).astype(np.float64)
        rng = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(rng > 0.5) & (rng < max_range)]
        p3 = np.c_[pts, np.zeros(len(pts)), np.ones(len(pts))].T
        t = np.eye(4)
        t[:3] = pose
        l = np.eye(4)
        l[:3] = local
        w = (t @ l @ p3)[:2].T
        sensor = (t @ l)[:2, 3]
        origins.append(np.repeat(sensor[None], len(w), 0))
        ends.append(w)
    origins = np.concatenate(origins)
    ends = np.concatenate(ends)
    print(f'{len(ends)} scan points')

    lo = np.minimum(ends.min(0), origins.min(0)) - 1.0
    hi = np.maximum(ends.max(0), origins.max(0)) + 1.0
    w, h = np.ceil((hi - lo) / res).astype(int)
    hits = np.zeros(w * h, np.int32)
    free = np.zeros(w * h, np.int32)

    def cells(xy):
        ij = np.floor((xy - lo) / res).astype(int)
        return ij[:, 1] * w + ij[:, 0]

    np.add.at(hits, cells(ends), 1)
    # Free space: sample every ray at half-cell steps, stopping one cell short of the hit.
    chunk = 200000
    for s in range(0, len(ends), chunk):
        o, e = origins[s:s + chunk], ends[s:s + chunk]
        d = e - o
        n = (np.hypot(d[:, 0], d[:, 1]) / (res / 2)).astype(int)
        steps = np.arange(n.max())
        for k in range(0, len(steps), 64):
            ks = steps[k:k + 64]
            mask = ks[None, :] < (n[:, None] - 2)
            f = (ks[None, :] / np.maximum(n[:, None], 1))[mask]
            rows = np.nonzero(mask)[0]
            xy = o[rows] + d[rows] * f[:, None]
            free += np.bincount(cells(xy), minlength=w * h)

    hits = hits.reshape(h, w)
    free = free.reshape(h, w)
    occ = hits >= 2
    occ &= hits > 0.1 * free  # cells mostly seen through are dynamic (people)
    known = (hits + free) > 0
    img = np.full((h, w), 205, np.uint8)
    img[known] = 254
    img[occ] = 0
    img = np.flipud(img)

    Image.fromarray(img).save(os.path.join(out_dir, name + '.pgm'))
    with open(os.path.join(out_dir, name + '.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    preview = np.stack([img] * 3, -1)
    traj = poses[:, :2, 3]
    tj = np.floor((traj - lo) / res).astype(int)
    for x, y in tj:
        preview[h - 1 - y, x] = (220, 30, 30)
    Image.fromarray(preview).save(os.path.join(out_dir, name + '_preview.png'))
    print(f'map {w}x{h} cells, {w * res:.1f} x {h * res:.1f} m, origin {lo}')


if __name__ == '__main__':
    main()
