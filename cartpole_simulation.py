import numpy as np
from backend import *

np_integer = lambda x: np.array(x, dtype=np.int32).copy()
np_real = lambda x: np.array(x, dtype=np.float64).copy()

def create_sim():
    sim = Simulator2d()
    sim.AddLink(1., [1.], [0, 0], -1, "prismatic", [0, 0], [0], [[1, 0]])
    sim.AddLink(1., [1.], [1.5, 0], 0, "hinge", [0, 0], [0], [])
    return sim

def simulation(save_file_prefix="cartpole", frame_num=500):
    sim = create_sim()
    q_list = [sim.q()]
    for i in range(frame_num):
        sim.ResetExternalForceAndTorque()
        sim.ApplyForce(1, [0, -9.8], sim.link_pose(1).translation())
        sim.Forward(0.01)
        q_list.append(sim.q())
    np.save(save_file_prefix + "q.npy", np_real(q_list))

if __name__ == "__main__":
    simulation("cartpole_gt_", 50)