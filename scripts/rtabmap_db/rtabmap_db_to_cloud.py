"""Export a colored 3D point cloud (PLY) from the RGB-D frames of an RTAB-Map
database, using the optimized poses (or a re-optimized graph if none are saved).

Usage: python -I rtabmap_db_to_cloud.py <map.db> <out.ply> [max_depth] [pixel_step] [voxel]
"""
import io
import sqlite3
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, __import__('os').path.dirname(__file__))
from clean_ply_cloud import write_ply
from rtabmap_db_to_grid import optimize_graph, uncompress


def main():
    db, out = sys.argv[1:3]
    max_depth = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 4
    voxel = float(sys.argv[5]) if len(sys.argv) > 5 else 0.03
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)

    ids_blob, poses_blob = con.execute('select opt_ids, opt_poses from Admin').fetchone()
    if ids_blob is None:
        ids, poses = optimize_graph(con)
    else:
        ids = uncompress(ids_blob)[0].reshape(-1)
        poses = uncompress(poses_blob)[0].reshape(-1, 3, 4).astype(np.float64)

    xyz, rgb = [], []
    for node_id, pose in zip(ids, poses):
        row = con.execute('select image, depth, calibration from Data where id=?', (int(node_id),)).fetchone()
        if not row or row[0] is None or row[1] is None:
            continue
        img = np.asarray(Image.open(io.BytesIO(row[0])).convert('RGB'))
        d = np.ascontiguousarray(np.asarray(Image.open(io.BytesIO(row[1]))))
        d = np.nan_to_num(d.view(np.float32)[..., 0] if d.ndim == 3 else d / 1000.0)
        k = np.frombuffer(row[2][44:116], np.float64).reshape(3, 3)
        local = np.eye(4)
        local[:3] = np.frombuffer(row[2][-48:], np.float32).reshape(3, 4)
        t = np.eye(4)
        t[:3] = pose
        v, u = np.mgrid[0:d.shape[0]:step, 0:d.shape[1]:step].reshape(2, -1)
        z = d[v, u]
        m = (z > 0.3) & (z < max_depth)
        v, u, z = v[m], u[m], z[m]
        cam = np.c_[(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z, np.ones(len(z))]
        xyz.append((t @ local @ cam.T)[:3].T)
        rgb.append(img[v, u])
    xyz = np.concatenate(xyz)
    rgb = np.concatenate(rgb)
    print(f'{len(xyz)} raw points')

    keys, inv = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    n = np.bincount(inv)
    xyz = np.stack([np.bincount(inv, xyz[:, i]) / n for i in range(3)], 1)
    rgb = np.stack([np.bincount(inv, rgb[:, i]) / n for i in range(3)], 1).astype(np.uint8)
    write_ply(out, xyz.astype(np.float32), rgb)
    print(f'{len(xyz)} points after {voxel} m voxel -> {out}')


if __name__ == '__main__':
    main()
