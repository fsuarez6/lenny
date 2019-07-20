#! /usr/bin/env python
import sys
import time
import random
import argparse
import itertools
import numpy as np
import baldor as br
import raveutils as ru
import robotsp as rtsp
import openravepy as orpy
from scipy.spatial import ConvexHull

from lenny_openrave.manager import EnvironmentManager
from lenny_openrave.scheduler import PDPScheduler
from lenny_openrave.bimanual import BimanualPlanner

def parse_args():
    # Remove extra IPython notebook args
    clean_argv = sys.argv
    if "-f" in clean_argv:
        clean_argv = clean_argv[1:]
    # Parse
    format_class = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(description="Lenny RoboTSP bimanual")
    parser.add_argument("-r", "--reachability", action="store_true", help="If set, will only show reachability")
    parser.add_argument("-s", "--seed", type=int, default=123, help="Seed for the random yaw of the cubes")
    parser.add_argument("-v", "--viewer", action="store_true", help="If set, will show the qtcoin viewer")
    parser.add_argument("--ik", action="store_true", help="If set, will only show the found IK solutions")
    return parser.parse_args()

args = parse_args()


np.set_printoptions(precision=6, suppress=True)
# Load the environment
env = orpy.Environment()
world_xml = "worlds/bimanual_pick_and_place.env.xml"
if not env.Load(world_xml):
  raise Exception("Failed to load world: {}".format(world_xml))
robot = env.GetRobot("robot")
robot.SetDOFValues(np.zeros(robot.GetDOF()))

# Environment manager
eman = EnvironmentManager(env, num_cubes=8, seed=args.seed)
cubes = eman.get_cubes()
bins = eman.get_bins()

# Initialize the bimanual planner
bimanual = BimanualPlanner(robot)
bimanual.set_left_manipulator("arm_left_tool0")
bimanual.set_right_manipulator("arm_right_tool0")
bimanual.set_torso_joint("torso_joint_b1")
if not bimanual.load_ikfast(freeinc=np.pi / 6.):
    print("Failed to load IKFast. Run the generate_robot_databases.sh script.")
    exit(0)

# Start and configure the viewer
if args.viewer:
    viewer_name = "qtcoin"
    print("Starting viewer: {0}".format(viewer_name))
    eman.start_viewer(viewer_name)

if args.reachability:
    # Reachability. One arm is enough due to symmetry
    print ("Computing reachable workspace...")
    indices = np.zeros(3)
    indices[1:] = bimanual.left_indices[:2]
    manip = bimanual.left_manip
    Toffset = np.eye(4)
    Toffset[:3,3] = [0., 0., -0.07]
    angles_list = []
    num_points = 1
    for qmin, qmax in zip(robot.GetDOFLimits(indices)[0], robot.GetDOFLimits(indices)[1]):
        angles_list.append(np.arange(qmin, qmax, np.deg2rad(15)))
        num_points = num_points * len(angles_list[-1])
    points = np.zeros((num_points, 3))
    idx = 0
    for angles in itertools.product(*angles_list):
        with robot:
            robot.SetDOFValues(angles, indices)
            T = np.dot(manip.GetEndEffectorTransform(), Toffset)
        points[idx] = T[:3,3]
        idx += 1
    vertices, faces = ru.mesh.trimesh_from_point_cloud(points)
    orange = np.array([ 1. ,0.54902,  0., 0.25])
    ocd = False
    if ocd:
        # Dependencies: $ pip install --user trimesh && sudo apt-get install openscad
        import trimesh
        # Remove the env bodies from the reachable mesh
        reachable = trimesh.Trimesh(vertices=vertices, faces=faces)
        for body in env.GetBodies():
            if body.IsRobot():
                continue
            corners = ru.body.get_bounding_box_corners(body)
            vertices, faces = ru.mesh.trimesh_from_point_cloud(corners)
            mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
            reachable = reachable.difference(mesh, 'scad')
        
        h = env.drawtrimesh(reachable.vertices, indices=reachable.faces, colors=orange)
    else:
        h = env.drawtrimesh(vertices, indices=faces, colors=orange)
    # Debug?
    import IPython
    IPython.embed(banner1="")
    exit(0)

# PDP Scheduler
num_cubes = len(cubes)
scheduler = PDPScheduler(bimanual)
print ("Seaching for reachable configurations...")
starttime  = time.time()
reachable_bins, reachable_cubes =scheduler.find_reachable_configurations(bins, cubes, freeinc=None)
sequence = scheduler.generate_sequence_random(reachable_cubes.keys(), num_cubes)
duration_step1 = time.time() - starttime
if sequence is None:
    print("Failed to find a valid sequence to pickup the cubes")
    exit(0)

# # TODO: Move this section to a class
starttime = time.time()
print ("Solving the problem...")
import networkx as nx
import raveutils as ru
# Robotsp from home, then to the cubes and finally back home again
qhome = np.zeros(robot.GetActiveDOF())
vmax = robot.GetDOFVelocityLimits(robot.GetActiveDOFIndices())
setslist = [[qhome]]
# Add the cubes and bins configurations.
for cube_i, cube_j in sequence:
    # Cubes
    cubes_configs = reachable_cubes[(cube_i, cube_j)]
    setslist.append(cubes_configs)
    # Bins
    bins_configs = reachable_bins[(cubes[cube_i], cubes[cube_j])]
    setslist.append(bins_configs)
setslist += [[qhome]]
cgraph, sets = rtsp.construct.from_sorted_setslist(setslist, distfn=rtsp.metric.max_joint_diff_fn, args=(1./vmax,))
ctour = nx.dijkstra_path(cgraph, source=0, target=cgraph.number_of_nodes()-1)
duration_step2 = time.time() - starttime

if args.ik:
    print("Showing the found robot configurations...")
    for idx in xrange(len(ctour)-1):
        raw_input("Press any key to continue...")
        u = ctour[idx]
        v = ctour[idx+1]
        qgoal = cgraph.node[v]["value"]
        bimanual.update_active_joints()
        with env:
            robot.SetActiveDOFValues(qgoal)
        twist = np.zeros(6)
        twist[2] = -0.085
        cpu_times = []
        trajectories = []
        for manip in [bimanual.left_manip, bimanual.right_manip]:
            robot.SetActiveManipulator(manip)
            robot.SetActiveDOFs(manip.GetArmIndices())
            starttime = time.time()
            traj = ru.planning.plan_cartesian_twist(robot, twist)
            cpu_times.append(time.time() - starttime)
            if traj is not None:
                trajectories.append(traj)
                ros_traj = ru.planning.ros_trajectory_from_openrave(robot.GetName(), traj)
                qarm = ros_traj.points[-1].positions
                robot.SetActiveDOFValues(qarm)
    import IPython
    IPython.embed(banner1="")
    exit(0)


# Compute the trajectories
cpu_times = []
trajectories = []
for idx in xrange(len(ctour)-1):
    u = ctour[idx]
    v = ctour[idx+1]
    qstart = cgraph.node[u]["value"]
    qgoal = cgraph.node[v]["value"]
    starttime = time.time()
    with robot:
        robot.SetActiveDOFValues(qstart)
        traj = bimanual.plan(qgoal, max_iters=150, max_ppiters=60)
    cpu_times.append(time.time() - starttime)
    if traj is None:
        break
    trajectories.append(traj)
raw_input("Press any key to visualize the trajectories...")
# Playback the trajectories
for i, traj in enumerate(trajectories):
    controller = robot.GetController()
    controller.SetPath(traj)
    robot.WaitForController(0)
    # Delete the cubes for every even trajectory (except for the last one)
    seq_index, remainder = divmod(i, 2)
    if (remainder == 0) and (i != len(trajectories) - 1):
        for name in sequence[seq_index]:
            with env:
                env.RemoveKinBodyByName(name)

duration_step3 = sum(cpu_times)

import IPython
IPython.embed(banner1="")
exit(0)


for u,v in sequence:
    try:
        msg = "{0}: {1}".format(u, tgraph.node[u]["type"])
        print(msg)
        msg = "{0}: {1}".format(v, tgraph.node[v]["type"])
        print(msg)
    except KeyError:
        pass


Tleft = robot.SetActiveManipulator("arm_left_tool0").GetTransform()
Tright = robot.SetActiveManipulator("arm_right_tool0").GetTransform()
Toffset = np.eye(4)
Toffset[:3,3] = [0., -0.011115027122497567, -0.14]
Tleft = np.dot(Tleft, Toffset)
Tright = np.dot(Tright, Toffset)
h1 = ru.visual.draw_axes(env, Tleft)
h2 = ru.visual.draw_axes(env, Tright)