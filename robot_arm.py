import open3d as o3d
from matplotlib.cm import Set1, Set2, Set3
from copy import deepcopy
color_list = [Set3.colors[1], Set1.colors[-1], Set3.colors[-1]]

from backend import *
import pymeshlab as pm
def compute_dynamics(name, density=1e3):
    mesh = pm.MeshSet()
    mesh.load_new_mesh(name)
    stat = mesh.apply_filter("get_geometric_measures")
    com = stat["center_of_mass"]
    mesh.apply_filter("compute_matrix_from_translation", axisx=-com[0], axisy=-com[1], axisz=-com[2])
    stat = mesh.apply_filter("get_geometric_measures")
    inertia = stat['inertia_tensor'] * density
    mass = stat['mesh_volume'] * density
    return mass, inertia, com
sim = Simulator3d()
robot_arm = compute_dynamics("mesh/robot_arm.obj")
robot_gripper = compute_dynamics("mesh/robot_gripper.obj")
sim.AddLink(*robot_arm, -1, "universal", [0, 0, 0.3], [0, 0], [[1, 0, 0], [0, 0, 1]])
sim.AddLink(*robot_gripper, 0, "hinge", [0, 0, 0], [0], [[1, 0, 0]])

vis = o3d.visualization.Visualizer()
vis.create_window(visible=True)
mesh_list = ["robot_arm", "robot_gripper", "robot_base"]
o3d_mesh = []
for c, m in zip(color_list, mesh_list):
    mesh = o3d.io.read_triangle_mesh("mesh/" + m + ".obj")
    if m[-1] == 'm':
        print(robot_arm[-1])
        mesh.translate(-robot_arm[-1])
    if m[-1] == 'r':
        print(robot_gripper[-1])
        mesh.translate(-robot_gripper[-1])
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(c)
    vis.add_geometry(mesh)
    o3d_mesh.append(mesh)

stored_view = \
'''
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 0.30000001192092896, 0.05000000074505806, 0.34999999403953552 ],
			"boundingbox_min" : [ -0.070000000298023224, -0.30807900428771973, -0.18736900389194489 ],
			"field_of_view" : 60.0,
			"front" : [ 0.748926402115348, -0.38991812820703342, 0.53579202822558281 ],
			"lookat" : [ 0.11020294599264935, 0.01014543605149788, 0.18931145644139563 ],
			"up" : [ -0.44800978698339411, 0.2978239830925184, 0.84296388170668446 ],
			"zoom" : 0.8600000000000001
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
'''

target_q = [-0.5, 3., 1.]

def control(sim):
    q = sim.q()
    q_dot = sim.q_dot()
    discrepancy = q - target_q
    # TODO: your job is to find a way to apply forces/torques so that the robot arm could reach the target pose.
    sim.ApplyTorque(0, [.5, -2., .5])


for i in range(100):
    vis.clear_geometries()
    for i, m in enumerate(o3d_mesh[:2]):
        pose = sim.link_pose(i)
        mesh = deepcopy(m).rotate(pose.rotation(), center=(0, 0, 0)).translate(pose.translation())
        vis.add_geometry(mesh)
    vis.add_geometry(o3d_mesh[-1])
    vis.set_view_status(stored_view)
    vis.poll_events()
    vis.update_renderer()
    sim.ResetExternalForceAndTorque()
    control(sim)
    sim.Forward(0.01)

print("Your discrepancy is: ", sim.q() - target_q)