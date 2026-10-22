"""Clean an RTAB-Map exported RGB point cloud (voxel filter + noise removal)
and render preview images.

Usage: python -I clean_ply_cloud.py <in.ply> <out_dir> <name> [voxel] [min_pts]
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

VERTEX = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                   ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])


def read_ply(path):
    with open(path, 'rb') as f:
        n = 0
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            if line == 'end_header':
                break
        return np.frombuffer(f.read(n * VERTEX.itemsize), dtype=VERTEX, count=n)


def write_ply(path, xyz, rgb):
    v = np.empty(len(xyz), VERTEX)
    v['x'], v['y'], v['z'] = xyz.T
    v['r'], v['g'], v['b'] = rgb.T
    with open(path, 'wb') as f:
        f.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {len(v)}\n'
                 'property float x\nproperty float y\nproperty float z\n'
                 'property uchar red\nproperty uchar green\nproperty uchar blue\n'
                 'end_header\n').encode())
        f.write(v.tobytes())


def main():
    src, out_dir, name = sys.argv[1:4]
    voxel = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
    min_pts = int(sys.argv[5]) if len(sys.argv) > 5 else 4
    os.makedirs(out_dir, exist_ok=True)

    v = read_ply(src)
    xyz = np.c_[v['x'], v['y'], v['z']].astype(np.float64)
    rgb = np.c_[v['r'], v['g'], v['b']].astype(np.float64)
    ok = np.isfinite(xyz).all(1)
    xyz, rgb = xyz[ok], rgb[ok]
    print(f'{len(xyz)} points, z percentiles 1/50/99: {np.percentile(xyz[:, 2], [1, 50, 99]).round(2)}')

    # Ceiling and below-floor points are almost all stereo noise.
    keep = (xyz[:, 2] > -0.2) & (xyz[:, 2] < 3.0)
    xyz, rgb = xyz[keep], rgb[keep]

    # Voxel average; drop voxels hit by too few points (isolated stereo noise).
    key = np.floor(xyz / voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(cnt), 6))
    np.add.at(sums, inv, np.c_[xyz, rgb])
    mean = sums / cnt[:, None]
    good = cnt >= min_pts
    out_xyz, out_rgb = mean[good, :3], mean[good, 3:].round().astype(np.uint8)
    print(f'{len(cnt)} voxels, {good.sum()} kept (>= {min_pts} pts)')
    write_ply(os.path.join(out_dir, name + '.ply'), out_xyz, out_rgb)

    c = out_rgb / 255.0
    order = np.argsort(out_xyz[:, 2])
    fig, ax = plt.subplots(figsize=(14, 14), dpi=110)
    ax.scatter(out_xyz[order, 0], out_xyz[order, 1], c=c[order], s=0.4, marker='s', linewidths=0)
    ax.set_aspect('equal')
    ax.set_facecolor('black')
    ax.set_title(f'{name} top view ({voxel} m voxels)')
    fig.savefig(os.path.join(out_dir, name + '_top.png'), bbox_inches='tight')
    plt.close(fig)

    sub = np.random.default_rng(0).choice(len(out_xyz), min(len(out_xyz), 400000), replace=False)
    fig = plt.figure(figsize=(16, 11), dpi=110)
    ax = fig.add_subplot(projection='3d')
    p = out_xyz[sub]
    ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=c[sub], s=0.3, linewidths=0)
    span = (p.max(0) - p.min(0))
    ax.set_box_aspect(span)
    ax.view_init(elev=35, azim=-60)
    ax.set_title(f'{name} 3D view')
    fig.savefig(os.path.join(out_dir, name + '_3d.png'), bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
