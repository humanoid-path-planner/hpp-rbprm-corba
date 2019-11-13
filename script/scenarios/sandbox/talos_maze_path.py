from hpp.corbaserver.rbprm.talos_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
import numpy as np
from pinocchio import Quaternion
import time

Robot.urdfName += "_large"

vMax = 0.5# linear velocity bound for the root
aMax = 0.7# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
rbprmBuilder.setJointBounds ("root_joint", [0,18.5, 0, 24., rbprmBuilder.ref_height, rbprmBuilder.ref_height])
rbprmBuilder.setJointBounds ('torso_1_joint', [0,0])
rbprmBuilder.setJointBounds ('torso_2_joint', [0,0])


# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(['talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-4,4,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",mu)
ps.setParameter("Kinodynamic/forceYawOrientation",True)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",500)
# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 5.])
afftool.loadObstacleModel ("hpp_environments", "multicontact/maze_hard", "planning", vf)
#load the viewer
try :
    v = vf.createViewer(displayArrows = True)
except Exception:
    print "No viewer started !"
    class FakeViewer():
        def __init__(self):
            return
        def __call__(self,q):
            return
        def addLandmark(self,a,b):
            return
    v = FakeViewer()
v.addLandmark(v.sceneName,1)
afftool.visualiseAffordances('Support', v, v.color.lightBrown)

q_init = rbprmBuilder.getCurrentConfig ();
q_init[0:3] = [0.1,3,rbprmBuilder.ref_height]
q_init[3:7] = [0,0,0,1]
q_init[-6:-3] = [0.01,0,0]
v(q_init)
# sample random position on a circle of radius 2m
q_goal = q_init[::]
q_goal[0:2] = [17.8,15]
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)



# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.selectPathValidation("RbprmPathValidation",0.05)
ps.addPathOptimizer ("RandomShortcutDynamic")
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")

t = ps.solve ()
print "Guide planning time : ",t
#v.solveAndDisplay("rm",10,0.01)

for i in range(20):
    print "Optimize path, "+str(i+1)+"/20 ... "
    ps.optimizePath(ps.numberPaths()-1)
pathId = ps.numberPaths()-1


# display solution : 
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
pp.dt=0.01
pp.displayPath(pathId)
#v.client.gui.setVisibility("path_"+str(pathId)+"_root","ALWAYS_ON_TOP")
pp.dt=0.01
#pp(pathId)

#v.client.gui.writeNodeFile("path_"+str(pathId)+"_root","guide_path_maze_hard.obj")

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

