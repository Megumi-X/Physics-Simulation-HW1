import numpy as np
from tqdm import tqdm
from scipy.spatial.transform import Rotation
import pymeshlab as pm
from backend import *

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

def compute_dynamics(v, e, density=1e3):
    mesh = pm.MeshSet()
    mesh.add_mesh(pm.Mesh(v, e))
    stat = mesh.apply_filter("get_geometric_measures")
    inertia = stat['inertia_tensor'] * density
    mass = stat['mesh_volume'] * density
    return mass, inertia

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

def create_sim():
    [r1_v, r1_e], [r2_v, r2_e], [r3_v, r3_e], [gyro_v, gyro_e] = create_mesh()
    # Setting up the simulator.
    sim = Simulator3d()
    sim.AddLink(*compute_dynamics(r1_v, r1_e), np.zeros(3), -1, "ball", np.zeros(3), np.zeros(3), [])
    sim.AddLink(*compute_dynamics(r2_v, r2_e), np.zeros(3), 0, "hinge", np.zeros(3), [0], [[1, 0, 0]])
    sim.AddLink(*compute_dynamics(r3_v, r3_e), np.zeros(3), 1, "hinge", np.zeros(3), [0], [[0, 0, 1]])
    sim.AddLink(*compute_dynamics(gyro_v, gyro_e, 3e3), np.zeros(3), 2, "hinge", np.zeros(3), [0], [[0, 1, 0]])
    return sim

def simulation(with_init_angular_momentum=False, store_file_prefix="", frame_num=600000):
    np.random.seed(42)
    sim = create_sim()
    # Initialize q0.
    q0 = np_real(np.zeros(6))
    q0_dot = np_real(np.zeros(6))
    # Initialize the three hinge joints.
    for j in range(1, 4):
        theta = np.random.uniform(low=0, high=np.pi * 2)
        q0[2 + j] = theta
        # Give the last link a speed.
        if j == 3 and with_init_angular_momentum:
            q0_dot[5] = np.pi * 4
    sim.set_q(q0)
    sim.set_q_dot(q0_dot)
    q_list = [q0]
    q_dot_list = [q0_dot]
    rotation_list = []
    print("Simulating ...")
    for i in tqdm(range(frame_num)):
        # Apply external torque.
        sim.ResetExternalForceAndTorque()
        if i < 150000:
            sim.ApplyTorque(0, [50, 40, 10])
        sim.Forward(1e-5)
        q_list.append(sim.q())
        q_dot_list.append(sim.q_dot())
        rotation_list.append([sim.link_pose(j).rotation() for j in range(4)])
    np.save(store_file_prefix + "q.npy", np_real(q_list))
    np.save(store_file_prefix + "q_dot.npy", np_real(q_dot_list))
    np.save(store_file_prefix + "r.npy", np_real(rotation_list))
    print("Simulation finished.")


if __name__ == "__main__":
    # For reproducing the mp4.
    simulation(False, "w0_")
    simulation(True, "w1_")
    # For generating the gt file.
    # simulation(False, "gyroscope_gt_", 5)
