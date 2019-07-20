#! /usr/bin/env python
import time
import itertools
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy


class EnvironmentManager(object):
    COLORS = {
        "RED": [0.8, 0., 0., 1.],
        "BLUE": [0., 0., 0.8, 1.],
        "GREEN": [0., 0.8, 0., 1.],
    }

    def __init__(self, env, num_cubes=6, seed=111):
        # Working entities
        self.env = env
        self.bins = self._add_bins()                    # keys: bin_type, values: bin_name
        self.cubes = self._add_cubes(seed)   # keys: cube_name, values: cube_type
        print("Environment manager successfully initialized")
    
    def _add_cubes(self, seed):
        table = self.env.GetKinBody("cubes_table")
        aabb = table.ComputeAABB()
        xdim, _, zdim = 2 * aabb.extents()
        Tabove_table = table.GetTransform()
        Tabove_table[:3, 3] += [0, 0, zdim + br._EPS]
        zcube = Tabove_table[2, 3] + 1e-5  # Small delta to avoid colliding with the table
        xx = np.linspace(0.35, 0.7, num=3)
        yy = np.linspace(-0.3, 0.3, num=5) - Tabove_table[1,3]
        num_cubes = 14
        np.random.seed(seed)
        yaws = (2 * np.random.rand(num_cubes) - 1) * np.deg2rad(45)
        count = itertools.count(1)
        color_names = [
            "RED", "GREEN", "BLUE", "GREEN", "BLUE", 
            "RED", "RED", "GREEN", "BLUE", "RED",
            "BLUE", "GREEN", "GREEN", "BLUE"
        ]
        cubes = dict()
        for i, (xcube, ycube) in enumerate(itertools.product(xx, yy)):
            if i >= num_cubes:
                break
            cube_name = "cube_{0:02d}".format(count.next())
            cube = self.env.ReadKinBodyXMLFile("objects/wood_cube.kinbody.xml")
            yaw = yaws[i]
            Tcube = br.euler.to_transform(0, 0, yaw)
            Tcube[:3, 3] = [xcube, ycube, zcube]
            with self.env:
                cube.SetName(cube_name)
                self.env.Add(cube)
                cube.SetTransform(Tcube)
            # Assign a color for the cube
            color_name = color_names[i]
            ru.body.set_body_color(cube, diffuse=self.COLORS[color_name])
            cubes[cube_name] = color_name
        return cubes
    
    def _add_bins(self):
        bins_table = self.env.GetKinBody("bins_table")
        aabb = bins_table.ComputeAABB()
        xdim, _, zdim = 2 * aabb.extents()
        Tabove_table = bins_table.GetTransform()
        Tabove_table[:3, 3] += [0, 0.6 - Tabove_table[1,3], zdim + br._EPS]
        offset = xdim / 2. - 0.175
        direction = Tabove_table[:3, 0]
        placements = [-1., 0, 1.]
        bins = dict()
        for i, (placement, color_name) in enumerate(zip(placements, self.COLORS.keys())):
            body = self.env.ReadKinBodyXMLFile("objects/plastic_bin.kinbody.xml")
            Tbody = np.array(Tabove_table)
            Tbody[:3, 3] += offset * direction * placement
            bin_name = "bin_{0:02d}".format(i+1)
            with self.env:
                body.SetName(bin_name)
                self.env.Add(body)
                body.SetTransform(Tbody)
            ru.body.set_body_color(body, diffuse=self.COLORS[color_name])
            bins[color_name] = bin_name
        return bins

    def get_bins(self):
        return self.bins

    def get_cubes(self):
        return self.cubes
    
    def start_viewer(self, viewer_name, size=(1024,768), Tcamera=None):
        self.env.SetViewer(viewer_name)
        while (self.env.GetViewer() is None):
            time.sleep(0.1)
        viewer = self.env.GetViewer()
        viewer.SetSize(*size)
        if Tcamera is None:
            rpy = np.deg2rad([235., 0., 110.])
            Tcamera = br.euler.to_transform(*rpy)
            Tcamera[:3,3] = [2.336, 0.83, 2.13]
        viewer.SetCamera(Tcamera)