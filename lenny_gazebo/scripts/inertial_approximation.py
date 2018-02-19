#! /usr/bin/env python
template = """
name = "{name}"
<inertial>
  <origin xyz="0 0 {z}" rpy="0 0 0" />
  <mass value="{mass}" />
  <inertia  ixx="{ixx}" ixy="0" ixz="0"
            iyy="{iyy}" iyz="0"
            izz="{izz}" />
</inertial>
"""

class Inertial(object):
  def __init__(self, name, mass, radius, length, reflect=False):
    self.name = name
    self.mass = mass
    self.radius = radius
    self.length = length
    self.reflect = reflect

  def __repr__(self):
    return '<Inertial name="{0}">'.format(self.name)

  def __str__(self):
    z = self.length / 2.
    if self.reflect:
      z = -z
    ixx = iyy = (self.mass/12.)*(3*self.radius**2  + self.length**2)
    izz = (self.mass*self.radius**2) / 2.
    res = template.format(name=self.name, z=z, mass=self.mass, ixx=ixx, iyy=iyy,
                                                                        izz=izz)
    return res

# The following values have been defined to add-up 220 Kg
links = []
links.append(Inertial('torso_base_link', mass=10, radius=0.2, length=0.86))
links.append(Inertial('torso_link_b1', mass=25, radius=0.15, length=1.354-0.86))
# We assume that all the arm links have the same dimensions
links.append(Inertial('link_1_s', mass=7, radius=0.146/2., length=0.25,
                                                                  reflect=True))
links.append(Inertial('link_2_l', mass=6, radius=0.125, length=0.146/2.,
                                                                  reflect=True))
links.append(Inertial('link_3_e', mass=5, radius=0.146/2., length=0.25,
                                                                  reflect=True))
links.append(Inertial('link_4_u', mass=5, radius=0.125, length=0.146/2.,
                                                                  reflect=True))
links.append(Inertial('link_5_r', mass=4, radius=0.146/2., length=0.25,
                                                                  reflect=True))
links.append(Inertial('link_6_b', mass=4, radius=0.125, length=0.146/2.,
                                                                  reflect=True))
links.append(Inertial('link_7_t', mass=0.5, radius=0.04,   length=0.015,
                                                                  reflect=True))
links.append(Inertial('coupler', mass=0.168, radius=0.1375, length=0.08,
                                                                  reflect=True))
for l in links:
  print l
