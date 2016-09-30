from hpp.corbaserver.rbprm.problem_solver import ProblemSolver as ProblemSolverRbprm
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver import Client
from hpp.gepetto import ViewerFactory
from hpp.gepetto import Viewer
import sys

from hpp.corbaserver.manipulation.romeo import Robot
from hpp.corbaserver.manipulation import ProblemSolver, Rule
from hpp.gepetto.manipulation import Viewer, ViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui
from math import sqrt


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
ps0 = ProblemSolver (robot)
#~ r = Viewer (ps)
vf = ViewerFactory (ps0)

robot.setJointBounds ("romeo/base_joint_xyz" , [-60,20,-50,100, 0, 2])

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"



#~ ps = ProblemSolver( fullBody )
#~ vf = ViewerFactory (ps)
#~ r = Viewer (ps)



robot.setJointBounds ("romeo/base_joint_xyz" , [-60,20,-50,100, 0, 2])
vf.loadObjectModel (Kitchen, "kitchen_area")
vf.loadObjectModel (Cup, "cup")

robot.setJointBounds ('cup/base_joint_xyz', [-60,20,-50,100, 0, 2])
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


# Create a new manipulation problem
cl = Client()
cl.problem.selectProblem("rbprm")
cl.problem.selectProblem("default")
cl.problem.moveRobotToProblem("rbprm") 
cl.problem.selectProblem("rbprm")

fullBody = FullBody ()

fullBody.loadFullBodyModelFromActiveRobot('romeo', {'cup': 'freeflyer', 'kitchen_area': 'anchor', 'romeo': 'freeflyer'}, "romeokitchen", 'romeo_description', '', '')
#~ fullBody.setJointBounds ("base_joint_xyz", [-4,4,-4,4,-4,4])

ps = ProblemSolverRbprm (robot)
r = Viewer (ps)
pp = PathPlayer (fullBody.client.basic, r)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel ("hpp_environments", "hrp2/floor_as_mesh", "floor", r)
#~ afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
fullBody.client.rbprm.rbprm.setAffordanceFilter('0rLeg', ['Support',])
fullBody.client.rbprm.rbprm.setAffordanceFilter('1lLeg', ['Support'])

import pickle

with open("romeo_kitchen_path_discretized.pickle", 'r') as f:
    qs = pickle.load(f)
fullBody.client.rbprm.rbprm.configToPath(qs)
#~ a = [q for i,q in enumerate(qs) if i % 50 == 0 ]

r.client.gui.addURDF("kitchen_area", "/home_local/dev/hpp/install/share/"+Kitchen.packageName+"/urdf/"+Kitchen.urdfName+".urdf", "")
r.client.gui.addToGroup("kitchen_area", r.sceneName)
r.client.gui.addURDF("cup", "/home_local/dev/hpp/install/share/"+Cup.packageName+"/urdf/"+Cup.urdfName+".urdf", "")
r.client.gui.addToGroup("cup", r.sceneName)


#~ r.loadObstacleModel (Kitchen.packageName, Kitchen.urdfName, "kitchen_area2")


	
#~ for j in fullBody.client.basic.obstacle.getObstacleNames(True, False):
	#~ if( j != 'floor/base_link_0'):
		#~ fullBody.client.basic.obstacle.removeObstacleFromJoint('floor/base_link_0', j, True, False)



#~ pp(0)

#~ print "addlef"
rLegId = '0rLeg'
rfoot = 'romeo/RAnkleRoll'
rLeg = 'romeo/RHipYaw'
rLegOffset = [0,0,-0.06839999246139947]
rLegNormal = [0,0,1]
rLegx = 0.1; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, 10000, "static", 0.05, "_6_DOF", True)
#~ 
#~ print "addlef"
lLegId = '1lLeg'
lLeg = 'romeo/LHipYaw'
lfoot = 'romeo/LAnkleRoll'
lLegOffset = [0,0,-0.06839999246139947]
lLegNormal = [0,0,1]
lLegx = 0.1; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, 10000, "static", 0.1, "_6_DOF", True)

#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)


fullBody.setStartState(qs[0],[rLegId,lLegId])
fullBody.setEndState(qs[20],[rLegId,lLegId])

#~ configs = fullBody.interpolate(0.15, 0, 10, True)
#~ 

for j in fullBody.getAllJointNames():
	#~ if j.startswith("kitchen") or j.startswith("cup"):
	fullBody.client.basic.obstacle.removeObstacleFromJoint('floor/base_link_0', j, True, False)
#~ ps.pathLength(0) / 100.
configs = fullBody.interpolate(ps.pathLength(0) / 50., 0, 10, True)

limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : rfoot},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : lLeg}, }

from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, verbose = False, draw = False, trackedEffectors = []):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale = 20., useCOMConstraints = True, use_window = use_window,
	verbose = verbose, draw = draw, trackedEffectors = trackedEffectors)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)
	
def draw_com():
	global fullBody
	c = fullBody.getCenterOfMass()
	scene = "com_" + str(c)
	r.client.gui.createScene(scene)
	r.client.gui.addBox(scene+"/b"+str(0),0.1,0.1,0.1, [1,0,0,1])
	r.client.gui.applyConfiguration(scene+"/b"+str(0),c+[1,0,0,0])
	r.client.gui.refresh()	
	r.client.gui.addSceneToWindow(scene,0)

#~ def playPaths(rs = None):
    #~ import time
    #~ ps.client.problem.selectProblem("rbprm")
    #~ ls = [  ps.pathLength(i) for i in range(ps.numberPaths()) ]
    #~ if rs is None:
        #~ rs = [ vf.createViewer() ]
        #~ ps.client.problem.selectProblem("manipulationProblem")
        #~ rs.append( manipulation.vf.createViewer() )
    #~ for i in range(1000):
        #~ ps.client.problem.selectProblem("rbprm")
        #~ rs[0] (ps.configAtParam(1,i * ls[1] / 1000.))
        #~ ps.client.problem.selectProblem("manipulationProblem")
        #~ rs[1] (manipulation.ps.configAtParam(0, i * ls[0] / 1000.))
        #~ time.sleep(0.5)
    #~ return rs
    
#~ for i in range(1,5):
    #~ act(i,60, use_window = 0, optim_effectors = True, draw = False, verbose = True)
    
#~ trackedEffectors = [0, 0, 0.15, ['LARM_JOINT5']]

#~ for i in range(0,1):
	#~ trackedEffectors = [0, i * 0.15, (i+1) * 0.15, ['LARM_JOINT5']];
	#~ act(i,60, use_window = 0, optim_effectors = True, draw = False, verbose = False, trackedEffectors = trackedEffectors)
	
#~ act(5,0, use_window = 0, friction = 1, optim_effectors = False, draw = False, verbose = True)


