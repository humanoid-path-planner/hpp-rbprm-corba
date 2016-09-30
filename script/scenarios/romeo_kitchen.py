# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.romeo import Robot
from hpp.corbaserver.manipulation import ProblemSolver, Rule
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt

# Load robot and object. {{{3

# Define classes for the objects {{{4
class Kitchen (object):
  rootJointType = "anchor"
  packageName = 'iai_maps'
  meshPackageName = 'iai_maps'
  urdfName = 'kitchen_area'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "kitchen_area/fridge_block_fridge_joint"
  handle = "kitchen_area/fridge_handle_fridge_handle"

class Cup (object):
  rootJointType = "freeflyer"
  packageName = 'hpp_tutorial'
  meshPackageName = 'hpp_tutorial'
  urdfName = 'cup'
  urdfSuffix = ""
  srdfSuffix = ""
  joint = "cup/base_joint"
  handle = "cup/handle"

Robot.srdfSuffix = "_moveit"
# 4}}}

robot = Robot ('romeo-kitchen', 'romeo')
ps = ProblemSolver (robot)
vf = ViewerFactory (ps)

robot.setJointBounds ("romeo/base_joint_xyz" , [-6,-2,-5.2,-2.7, 0, 2])
vf.loadObjectModel (Kitchen, "kitchen_area")
vf.loadObjectModel (Cup, "cup")

robot.setJointBounds ('cup/base_joint_xyz', [-6,-4,-5,-3,0,1.5])
# 3}}}

# Define configurations. {{{3
robot.setCurrentConfig (robot.getInitialConfig ())
q_init = robot.getHandConfig ("both", "open")
rank = robot.rankInConfiguration ['romeo/base_joint_xyz']
# q_init [rank:rank+7] = [-3.5,-3.7, 0.877, 1, 0, 0, 0]
q_init [rank:rank+7] = [-4.264,-4.69, 0.877, 0, 0, 0, 1]
rank = robot.rankInConfiguration ['cup/base_joint_xyz']
q_init [rank:rank+7] = [-4.8, -4.64, 0.91,0,sqrt(2)/2,sqrt(2)/2,0]

q_goal1 = q_init [::]
q_goal2 = q_init [::]
q_goal1 [rank:rank+7] = [-4.73, -3.35, 0.91, 0,sqrt(2)/2,sqrt(2)/2,0]
q_goal2 [rank:rank+7] = [-4.8, -4.70, 0.91, 0,sqrt(2)/2,sqrt(2)/2,0]
# 3}}}
