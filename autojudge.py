from backend import *
import numpy as np

def check_problem1():
    test_funcs = [TestBallJoint, TestHingeJoint2d, TestHingeJoint3d, TestPlanarJoint, TestPrismaticJoint2d, TestPrismaticJoint3d, TestUniversalJoint]
    score = 0
    for func in test_funcs:
        try:
            for _ in range(10): func()
            score += 1
        except RuntimeError:
            print("")
            continue
    print(f"Numerical tests pass for ({score}/7) types of joints.")


def check_problem2():
    def check_cartpole():
        from cartpole_simulation import create_sim
        q = np.load("cartpole_gt_q.npy")
        frame_len = len(q)
        sim = create_sim()
        sim.set_q(q[0])
        q_list = [q[0]]
        for i in range(frame_len - 1):
            try:
                sim.ResetExternalForceAndTorque()
                sim.ApplyForce(1, [0, -9.8], sim.link_pose(1).translation())
                sim.Forward(0.01)
                q_list.append(sim.q())
            except RuntimeError:
                print("Error occurs in simulation.")
                return 0
        if not np.isclose(q, np.array(q_list)).all():
            print("Your implementation in 2D is possibly wrong.")
            return 0
        else:
            print("Your implementation in 2D is consistent with GT.")
            return 1


    def check_gyroscope():
        from gyroscope_simulation import create_sim
        q = np.load("gyroscope_gt_q.npy")
        frame_len = len(q)
        sim = create_sim()
        sim.set_q(q[0])
        q_list = [q[0]]
        for i in range(frame_len - 1):
            try:
                sim.ResetExternalForceAndTorque()
                sim.ApplyTorque(0, [50, 40, 10])
                sim.Forward(1e-5)
                q_list.append(sim.q())
            except RuntimeError:
                print("Error occurs in simulation.")
                return 0
        if not np.isclose(q, np.array(q_list)).all():
            print("Your implementation in 3D is possibly wrong.")
            return 0
        else:
            print("Your implementation in 3D is consistent with GT.")
            return 1

    score = 0
    score += check_cartpole()
    score += check_gyroscope()
    print(f"Tests for ({score}/2) scenes pass.")


if __name__ == "__main__":
    check_problem1()
    check_problem2()
