#! /usr/bin/env python
import itertools
import numpy as np
import baldor as br
import networkx as nx
import criutils as cu
import raveutils as ru
import robotsp as rtsp
import lkh_solver as lkh
import openravepy as orpy

# from lenny_openrave.manager import BottleManager


class PDPScheduler(object):
    def __init__(self, bimanual):
        self.bimanual = bimanual
        self.robot = bimanual.robot
        self.env = self.robot.GetEnv()
        self.bin_offset = np.array([0, 0, 0.25])
        self.cube_offset = np.array([0, 0, 0.22])
    
    def construct_pdp_graph(self, bins, cubes, distfn=rtsp.metric.euclidean_fn):
        # Construct the Multi-commodity pickup-and-delivery TSP (m-PDTSP) graph
        graph = nx.Graph()
        robot_position = self.robot.GetTransform()[:3, 3]
        graph.add_node("home", position=robot_position, demand=0)   # Robot home
        # Add cubes and bins nodes. We need to create dummy bins due to the m-PDTSP problem definition
        binid_list = []
        for cube_id, cube_type in cubes.items():
            # Cube
            Tcube = self.env.GetKinBody(cube_id).GetTransform()
            Ttcp_cube = np.array(Tcube)
            Ttcp_cube[:3, 3] += self.cube_offset
            graph.add_node(cube_id, position=Tcube[:3,3], demand=-1, type=cube_type, Ttcp=Ttcp_cube)
            # Bin
            bin_name = bins[cube_type]
            Tbin = self.env.GetKinBody(bin_name).GetTransform()
            Ttcp_bin = np.array(Tbin)
            Ttcp_bin[:3, 3] += self.bin_offset
            bin_id = cube_id.replace("cube", "bin")
            graph.add_node(bin_id, position=Tbin[:3,3], demand=1, type=cube_type, Ttcp=Ttcp_bin)
            binid_list.append(bin_id)
        # Compute the distances all the node pairs
        for u, v in itertools.combinations(graph.nodes_iter(), 2):
            dist = distfn(graph.node[u]["position"], graph.node[v]["position"])
            # if ("cube" in u and "cube" in v):
            #     dist = 5 * dist
            graph.add_edge(u, v, weight=dist)
        # Compute the set of valid color combinations for the 'place'
        # That is, valid pairs of bins for dropping the objects
        reachable_set = set()
        tmp_configs = dict()
        valid_bin_combinations = set()
        checked_combinations = set()
        nonedge = 99.999
        for node_i, node_j in itertools.combinations(binid_list, 2):
            type_i = graph.node[node_i]["type"]
            type_j = graph.node[node_j]["type"]
            if type_i == type_j:
                # Remove edges between bins of the same type, if any
                graph.add_edge(node_i, node_j, weight=nonedge)
                continue
            if (type_i, type_j) in valid_bin_combinations:
                combination = (type_i, type_j)
            else:
                combination = (type_j, type_i)
            num_configs = 0
            if combination in valid_bin_combinations:
                configs = tmp_configs[combination]
                num_configs = len(configs)
            else:
                Ti = graph.node[node_i]["Ttcp"]
                Tj = graph.node[node_j]["Ttcp"]
                crossed, configs = self.find_crossed_configurations(Ti, Tj)
                if configs is not None:
                    valid_bin_combinations.add(combination)
                    tmp_configs[combination] = configs
            if num_configs > 0:
                graph.add_edge(node_i, node_j, configs=configs, weight=0)
                reachable_set.add(node_i)
                reachable_set.add(node_j)
            debug_msg = "num configs {0}->{1}: {2}".format(node_i, node_j, num_configs)
        # Now, process the cubes
        valid_cube_combinations = set()
        for node_i, node_j in itertools.combinations(cubes.keys(), 2):
            type_i = graph.node[node_i]["type"]
            type_j = graph.node[node_j]["type"]
            if type_i == type_j:
                continue
            Ti = graph.node[node_i]["Ttcp"]
            Tj = graph.node[node_j]["Ttcp"]
            if (type_i, type_j) in valid_bin_combinations or (type_j, type_i) in valid_bin_combinations:
                crossed, configs = self.find_crossed_configurations(Ti, Tj)
                if configs is not None:
                    if crossed:
                        valid_cube_combinations.add((node_j, node_i))
                    else:
                        valid_cube_combinations.add((node_i, node_j))
                    graph.add_edge(node_i, node_j, weight=0, configs=configs)
                    reachable_set.add(node_i)
                    reachable_set.add(node_j)
        return graph, reachable_set
    
    def find_crossed_configurations(self, Ti, Tj):
        qtorso = self.bimanual.estimate_torso_angle(Ti[:3, 3], Tj[:3, 3])
        with self.robot:
            self.bimanual.set_torso_joint_value(qtorso)
            crossed = self.bimanual.lazy_crossed_arms_check(Ti[:3, 3], Tj[:3, 3])
        if crossed:
            with self.robot:
                self.bimanual.set_torso_joint_value(qtorso)
                solutions = self.bimanual.find_ik_solutions(Tj, Ti, freeinc=np.pi)
        else:
            with self.robot:
                self.bimanual.set_torso_joint_value(qtorso)
                solutions = self.bimanual.find_ik_solutions(Ti, Tj, freeinc=np.pi)
        num_sols = len(solutions)
        configs = None
        if num_sols > 0:
            configs = np.zeros( (num_sols, self.robot.GetActiveDOF()) )
            configs[:, 0] = qtorso
            configs[:, 1:] = solutions
        return crossed, configs
    
    def generate_sequence(self, graph, types):
        # Generate the demand matrix
        nodelist = sorted(graph.nodes())
        # Make sure that the first node is "home". Seems like the LKH solver is expecting the depot to be the first node.
        depot = 0
        nodelist.pop(nodelist.index("home"))
        nodelist.insert(depot, "home")
        num_nodes = graph.number_of_nodes()
        num_types = len(types)
        type_indices = {name:index for index, name in enumerate(types)}
        demand = np.zeros((num_nodes, num_types), dtype=int)
        for i in xrange(num_nodes):
            n = nodelist[i]
            node_demand = graph.node[n]["demand"]
            if node_demand == 0:
                continue
            type_name = graph.node[n]["type"]
            type_idx = type_indices[type_name]
            demand[i, type_idx] = node_demand
        if np.any(np.sum(demand, axis=0)):
            # The demand sum for each column must be zero
            raise Exception("The demand matrix is not feasible")
        # m-PDTSP
        params = lkh.solver.SolverParameters()
        m, n = num_types, num_nodes - num_types - 1
        params.problem_file = "/tmp/lkh/m{0}n{1}task.m-pdtsp".format(m, n)
        params.max_trials = 1
        params.runs = 3
        params.special = True
        params.depot = depot + 1
        lkh.parser.write_tsplib(params.problem_file, graph, params, nodelist=nodelist, demand=demand, capacity=2, depot=depot)
        tour, info = lkh.solver.lkh_solver(params)
        if tour is None:
            raise Exception("Failed to find a feasible sequence")
        indices = np.array(tour[0]) - 1
        if indices[0] != depot:
            raise Exception("The first node must be the depot/home")
        # Get the cubes sequence
        sequence = list()
        prev_node =  None
        for idx in indices:
            node = nodelist[idx]
            if "cube" not in node:
                prev_node = None
                continue
            if prev_node is None:
                prev_node = node
                continue
            sequence.append((prev_node, node))
            weight = graph.edge[prev_node][node]["weight"]
            if weight != 0:
                raise Exception("Unexpected weight {0}-{1}: {2}".format(prev_node, node, weight))
        return sequence