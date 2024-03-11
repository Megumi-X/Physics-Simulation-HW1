from matplotlib import pyplot as plt
from matplotlib import collections as mc
from cartpole_simulation import *

# TODO: this visualization in matplotlib is not good. Maybe we can improve it in the future.
def visualization(frame_num=500):
    sim = create_sim()
    for i in range(frame_num):
        sim.ResetExternalForceAndTorque()
        sim.ApplyForce(1, [0, -9.8], sim.link_pose(1).translation())
        sim.Forward(0.01)
        if i % 10 == 0:
            box1 = np_real([[-1, -.5], [1, -.5], [1, .5], [-1, .5]])
            box1 += sim.link_pose(0).translation()
            box2 = np_real([[-1.5, -.1], [1.5, -.1], [1.5, .1], [-1.5, .1]])
            box2_pose = sim.link_pose(1)
            box2 = box2 @ box2_pose.rotation().T + box2_pose.translation()
            fig = plt.figure()
            ax = fig.add_subplot()

            ax.add_collection(mc.PolyCollection([box1]))
            ax.add_collection(mc.PolyCollection([box2], color="tab:red"))
            ax.set_xlim([-3, 3])
            ax.set_ylim([-3, 2])
            ax.set_aspect("equal")
            ax.set_axis_off()
            plt.show()

if __name__ == "__main__":
    visualization()