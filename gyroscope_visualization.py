import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
import numpy as np
from matplotlib.cm import Set1, Set2, Set3
import open3d as o3d
from copy import deepcopy

path_prefix = "w0_"

np_integer = lambda x: np.array(x, dtype=np.int32).copy()
np_real = lambda x: np.array(x, dtype=np.float64).copy()

def create_ring(r_num, c_num, r, h):
    circle = np_real([[h * np.cos(t), 0, h * np.sin(t)] for t in np.linspace(0, np.pi * 2, c_num, 0)]) + [[r, 0, 0]]
    v = np.vstack([circle @ [[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]] for t in np.linspace(0, np.pi * 2, r_num, 0)])
    e = []
    for i in range(r_num):
        j = (i + 1) % r_num
        for k in range(c_num):
            l = (k + 1) % c_num
            e += [[i * c_num + k, i * c_num + l, j * c_num + k], [i * c_num + l, j * c_num + l, j * c_num + k]]
    return np_real(v), np_integer(e)

def create_rotor(radius):
    l = radius / 2
    v = [[l, 0, l], [-l, 0, l], [-l, 0, -l], [l, 0, -l], [0, radius, 0], [0, -radius, 0]]
    e = [[1, 0, 4], [2, 1, 4], [3, 2, 4], [0, 3, 4], [0, 1, 5], [1, 2, 5], [2, 3, 5], [3, 0, 5]]
    return np_real(v), np_integer(e)

def create_mesh():
    # Geometry.
    # The outmost ring.
    r1_v, r1_e = create_ring(256, 256, 0.95, 0.05)
    # The middle ring.
    r2_v, r2_e = create_ring(256, 256, 0.85, 0.05)
    r2_v = r2_v @ Rotation.from_rotvec([np.pi / 2, 0, 0]).as_matrix()
    # The inner ring.
    r3_v, r3_e = create_ring(4, 3, 0.7, 0.1)
    r3_v = r3_v @ Rotation.from_rotvec([0, np.pi / 2, 0]).as_matrix()
    # The gyro.
    gyro_v, gyro_e = create_rotor(0.65)
    return [r1_v, r1_e], [r2_v, r2_e], [r3_v, r3_e], [gyro_v, gyro_e]

mesh_list = create_mesh()
color_list = [Set3.colors[0], Set1.colors[4], Set3.colors[4], Set3.colors[6]]

def create_o3d(v, e):
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(e))
    mesh.compute_vertex_normals()
    return mesh

if not Path.exists(Path(path_prefix + "r.npy")):
    raise FileNotFoundError("Missing r.npy. Please run the gyroscope_simulation.py first, and specify the correct path prefix.")

r = np.load(path_prefix + "r.npy")

vis = o3d.visualization.Visualizer()
vis.create_window(visible=True)
o3d_mesh = []
for c, m in zip(color_list, mesh_list):
    mesh = create_o3d(*m)
    mesh.paint_uniform_color(c)
    o3d_mesh.append(mesh)

for i in range(len(r)):
    if i % 1000 == 0:
        vis.clear_geometries()
        for j, m in enumerate(o3d_mesh):
            rot = r[i, j, :, :]
            vis.add_geometry(deepcopy(m).rotate(rot, center=(0, 0, 0)))
        vis.poll_events()
        vis.update_renderer()
