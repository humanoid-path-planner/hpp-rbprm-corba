import config as cfg


#root path configuration

from hpp.corbaserver.rbprm.anymal_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
import numpy as np
import time
statusFilename = "/tmp/infos.log"


vMax = 0.3# linear velocity bound for the root
aMax = 1.# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
root_bounds =  cfg.ROOT_BOUNDS
rbprmBuilder.setJointBounds ("root_joint", root_bounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(rbprmBuilder.urdfNameRom)
for rom in rbprmBuilder.urdfNameRom :
    rbprmBuilder.setAffordanceFilter(rom, ['Support'])

# We also bound the rotations of the torso. (z, y, x)
#~ rbprmBuilder.boundSO3([-1.7,1.7,-0.1,0.1,-0.1,0.1])
rbprmBuilder.boundSO3([-3,3,-3.,3.,-3.,-3])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
psGuide = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
psGuide.setParameter("Kinodynamic/velocityBound",vMax)
psGuide.setParameter("Kinodynamic/accelerationBound",aMax)
psGuide.setParameter("DynamicPlanner/sizeFootX",0.01)
psGuide.setParameter("DynamicPlanner/sizeFootY",0.01)
psGuide.setParameter("DynamicPlanner/friction",mu)
# sample only configuration with null velocity and acceleration :
psGuide.setParameter("ConfigurationShooter/sampleExtraDOF",False)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (psGuide)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp_environments", cfg.SCENE, "planning", vf)
try :
    v = vf.createViewer(displayArrows = True)
except Exception:
    print "No viewer started !"
    class FakeViewer():
        def __init__(self):
            return
        def __call__(self,q):
            return
    v = FakeViewer()
    
#afftool.visualiseAffordances('Support', v, v.color.lightBrown)

q_init = rbprmBuilder.getCurrentConfig ();
q_init[:7] = cfg.INIT_CONFIG_ROOT


# Choosing RBPRM shooter and path validation methods.
psGuide.selectConfigurationShooter("RbprmShooter")
psGuide.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
psGuide.selectSteeringMethod("RBPRMKinodynamic")
psGuide.selectDistance("Kinodynamic")
psGuide.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :
# move the robot out of the view before computing the contacts

from hpp.corbaserver import Client
 #~ #DEMO code to play root path and final contact plan
cl = Client()
cl.problem.selectProblem("rbprm_path")
rbprmBuilder2 = Robot ("toto")
ps2 = ProblemSolver( rbprmBuilder2 )
cl.problem.selectProblem("default")
#~ cl.problem.movePathToProblem(pId,"rbprm_path",rbprmBuilder.getAllJointNames()[1:])
r2 = Viewer (ps2, viewerClient=v.client)
q_far = q_init[::]
q_far[2] = -2
r2(q_far)

from hpp.gepetto import PathPlayer
pp = PathPlayer (v)

###### WHOLE BODY INIT

from hpp.corbaserver.rbprm.anymal import Robot
#~ from hpp.gepetto import Viewer
from tools.display_tools import *
import time

pId = 0

fullBody = Robot ()
# Set the bounds for the root
fullBody.setJointBounds ("root_joint",  root_bounds)
## reduce bounds on joints along x, to put conservative condition on the contact generation for sideway steps
fullBody.setVeryConstrainedJointsBounds()
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
ps = ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
#load the viewer
try :
    v = Viewer (ps,viewerClient=v.client, displayCoM = True)
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


if cfg.INIT_CONFIG_WB is None:
    # load a reference configuration
    q_ref = fullBody.referenceConfig[::]+[0]*6
    #q_ref = fullBody.referenceConfig_legsApart[::]+[0]*6
else:
    q_ref = cfg.INIT_CONFIG_WB + [0]*6
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)
fullBody.usePosturalTaskContactCreation(True)


fullBody.setCurrentConfig (q_init)

print "Generate limb DB ..."
tStart = time.time()
# generate databases : 
"""
nbSamples = 100000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, heuristicR, 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, heuristicL, 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)
"""
#~ fullBody.loadAllLimbs("fixedStep04","ReferenceConfiguration")
fullBody.loadAllLimbs("static","ReferenceConfiguration")

tGenerate =  time.time() - tStart
print "Done."
print "Databases generated in : "+str(tGenerate)+" s"

#define initial and final configurations : 
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = cfg.INIT_CONFIG_ROOT[0:7] # use this to get the correct orientation
vel_init = [0,0,0]
acc_init = [0,0,0]
vel_goal = [0,0,0]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = vel_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
#~ q_goal[configSize:configSize+3] = vel_goal[::]
#~ q_goal[configSize+3:configSize+6] = [0,0,0]


q_init[2] = q_ref[2]
#~ q_goal[2] = q_ref[2]




fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
v(q_init)


v.addLandmark('anymal/base_0',0.3)
v(q_init)
#fullBody.setReferenceConfig(fullBody.referenceConfig_legsApart[::]+[0]*6)

fullBody.setStartState(q_init,[fullBody.rArmId,fullBody.rLegId,fullBody.lArmId,fullBody.lLegId],[[0.,0.,1.] for _ in range(4)])

from hpp.corbaserver.rbprm import rbprmstate
from hpp.corbaserver.rbprm.rbprmstate import State

initState = State(fullBody, fullBody.getNumStates()-1)


import hpp.corbaserver.rbprm.fewstepsplanner as sp

def dispContactPlan(states, step = 0.5):
	for s in states:
		v(s.q());
		time.sleep(step)

fewStepPlanner = sp.FewStepPlanner(cl,psGuide,rbprmBuilder, fullBody, pathPlayer = pp)
