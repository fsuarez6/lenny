#! /usr/bin/env python
import numpy as np
import baldor as br
import criutils as cu
import raveutils as ru
import openravepy as orpy


class PDPScheduler(object):
    def __init__(self, env, bimanual):
        self.env = env
        self.bimanual = bimanual
    
    def generate_sequence(self, bins, bottles):
        pass