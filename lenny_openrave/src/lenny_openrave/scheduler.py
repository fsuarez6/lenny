#! /usr/bin/env python
import random
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
        self.bin_offset = np.array([0, 0, 0.15])
        self.cube_offset = np.array([0, 0, 0.15])
    
    def get_bin_tcp_transform(self, bin_name):
        Tbin = self.env.GetKinBody(bin_name).GetTransform()
        Ttcp_bin = np.array(Tbin)
        Ttcp_bin[:3, 3] += self.bin_offset
        return Ttcp_bin
    
    def get_cube_tcp_transform(self, cube_name):
        Tbin = self.env.GetKinBody(cube_name).GetTransform()
        Ttcp_bin = np.array(Tbin)
        Ttcp_bin[:3, 3] += self.cube_offset
        return Ttcp_bin

    def find_reachable_configurations(self, bins, cubes, freeinc=None):
        unreachable_bin_combinations_set = set()
        reachable_bin_combinations = dict()
        reachable_cube_combinations = dict()
        for cube_i, cube_j in itertools.combinations(cubes.keys(), 2):
            type_i = cubes[cube_i]
            type_j = cubes[cube_j]
            combination = (type_i, type_j)
            if type_i == type_j or combination in unreachable_bin_combinations_set:
                # Ignore cubes of the same type or cubes that cannot be placed
                continue
            # Check if we can reach this bin combination
            reachable_bin_combination = True
            if combination not in reachable_bin_combinations:
                Tbin_i = self.get_bin_tcp_transform(bins[type_i])
                Tbin_j = self.get_bin_tcp_transform(bins[type_j])
                crossed, configs = self.bimanual.find_wholebody_ik_solutions(Tbin_i, Tbin_j, freeinc=freeinc)
                reachable_bin_combination = (not crossed) and (configs is not None)
                if configs is not None:
                    if crossed:
                        unreachable_bin_combinations_set.add(combination)
                        cache_key = tuple(reversed(combination))
                    else:
                        cache_key = tuple(combination)
                    reachable_bin_combinations[cache_key] = configs
            if not reachable_bin_combination:
                continue
            # Check if we can reach this pair of cubes
            Tcube_i = self.get_cube_tcp_transform(cube_i)
            Tcube_j = self.get_cube_tcp_transform(cube_j)
            crossed, configs = self.bimanual.find_wholebody_ik_solutions(Tcube_i, Tcube_j, freeinc=freeinc)
            reachable = (not crossed) and (configs is not None)
            if reachable:
                reachable_cube_combinations[(cube_i, cube_j)] = configs
        return reachable_bin_combinations, reachable_cube_combinations
    
    def generate_sequence_random(self, reachable_cube_pairs, num_cubes):
        sequence = None
        combinations = list(reachable_cube_pairs)
        random.shuffle(combinations)
        for pairs in itertools.combinations(combinations, num_cubes/2):
            visited = set()
            for cube_i, cube_j in pairs:
                if cube_i not in visited and cube_j not in visited:
                    visited.add(cube_i)
                    visited.add(cube_j)
            if len(visited) == num_cubes:
                sequence = sorted(pairs)
                break
        return sequence
    
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
                # Remove edges between bins of the same type
                graph.add_edge(node_i, node_j, weight=nonedge)
                continue
            combination = (type_i, type_j)
            num_configs = 0
            if combination in valid_bin_combinations:
                configs = tmp_configs[combination]
                num_configs = len(configs)
            elif combination not in checked_combinations:
                checked_combinations.add(combination)
                Ti = graph.node[node_i]["Ttcp"]
                Tj = graph.node[node_j]["Ttcp"]
                crossed, configs = self.find_crossed_configurations(Ti, Tj)
                if configs is not None:
                    if crossed:
                        combination = (type_j, type_i)
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
                # Remove edges between cubes of the same type
                graph.add_edge(node_i, node_j, weight=nonedge)
                continue
            Ti = graph.node[node_i]["Ttcp"]
            Tj = graph.node[node_j]["Ttcp"]
            if (type_i, type_j) in valid_bin_combinations:
                crossed, configs = self.find_crossed_configurations(Ti, Tj)
                if (not crossed) and (configs is not None):
                    valid_cube_combinations.add((node_i, node_j))
                    graph.add_edge(node_i, node_j, weight=0, configs=configs)
                    reachable_set.add(node_i)
                    reachable_set.add(node_j)
            weight = graph.edge[node_i][node_j]["weight"]
            if weight != 0:
                # Penalize unreachable cube pairs
                graph.add_edge(node_i, node_j, weight=nonedge)
        return graph, reachable_set
    
    def find_crossed_configurations(self, Ti, Tj):
        qtorso = self.bimanual.estimate_torso_angle(Ti[:3, 3], Tj[:3, 3])
        with self.robot:
            self.bimanual.set_torso_joint_value(qtorso)
            crossed = self.bimanual.lazy_crossed_arms_check(Ti[:3, 3], Tj[:3, 3])
        if crossed:
            with self.robot:
                self.bimanual.set_torso_joint_value(qtorso)
                solutions = self.bimanual.find_ik_solutions(Tj, Ti)
        else:
            with self.robot:
                self.bimanual.set_torso_joint_value(qtorso)
                solutions = self.bimanual.find_ik_solutions(Ti, Tj)
        num_sols = len(solutions)
        configs = None
        if num_sols > 0:
            configs = np.zeros( (num_sols, self.robot.GetActiveDOF()) )
            configs[:, 0] = qtorso
            configs[:, 1:] = solutions
        return crossed, configs
    
    def generate_sequence_pdtsp(self, graph, types):
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
                # raise Exception("Unexpected weight {0} -- {1}: {2}".format(prev_node, node, weight))
                print("Unexpected weight {0} -- {1}: {2}".format(prev_node, node, weight))
        return sequence