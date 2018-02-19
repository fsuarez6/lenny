#!/usr/bin/env python
import os
import rospy
import lxml.etree
import itertools
import numpy as np
import baldor as br
import criutils as cu
import resource_retriever
# Services and messages
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose, Point


if __name__ == '__main__':
  rospy.init_node('place_cubes_randomly')
  basename = 'cube'
  cube_z = 0.775
  xx = np.arange(0.5, 0.9, 0.1)
  yy = np.arange(-0.4, 0.5, 0.1)
  max_num_cubes = len(xx) * len(yy)
  # Read node parameters
  seed = rospy.get_param('~seed', None)
  num_cubes =  np.clip(rospy.get_param('~num_cubes', 20), 0, max_num_cubes)
  # Connect to the gazebo services and wait for them
  getprop_srv = rospy.ServiceProxy('/gazebo/get_world_properties',
                                                            GetWorldProperties)
  delete_srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
  spawn_srv = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
  getprop_srv.wait_for_service()
  delete_srv.wait_for_service()
  spawn_srv.wait_for_service()
  # try to delete any prior models in the simulation
  response = getprop_srv.call()
  for item_name in response.model_names:
    if basename in item_name:
      delete_srv.call(model_name=item_name)
  # Load the model sdf
  models_uri = 'package://lenny_gazebo/models'
  models_name = ['wood_cube_5cm', 'wood_cube_2_5cm']
  models_sdf = []
  models_size = []
  for name in models_name:
    uri = os.path.join(models_uri, name, 'model.sdf')
    path = resource_retriever.get_filename(uri, use_protocol=False)
    with open(path, 'r') as f:
      sdf = f.read()
    models_sdf.append(sdf)
    root = lxml.etree.fromstring(sdf)
    size_text = root.find('model/link/visual/geometry/box/size').text
    models_size.append(map(float, size_text.split(' ')))
  # Spawn the models
  np.random.seed(seed)
  indices = np.random.choice(np.arange(max_num_cubes), num_cubes, replace=False)
  yaws = (2*np.random.rand(max_num_cubes) - 1) * np.deg2rad(45)
  for i,(cube_x,cube_y) in enumerate(itertools.product(xx, yy)):
    if i not in indices:
      continue
    name = '{0}_{1:02d}'.format(basename, i)
    yaw = yaws[i]
    j = np.random.choice(range(len(models_sdf)))
    sdf = models_sdf[j]
    T = br.euler.to_transform(0, 0, yaw)
    T[:3,3] = [cube_x, cube_y, cube_z]
    pose = cu.conversions.to_pose(T)
    spawn_srv.call(model_name=name, model_xml=sdf, initial_pose=pose,
                                                        reference_frame='world')
