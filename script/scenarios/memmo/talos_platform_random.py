from hpp.corbaserver.rbprm.talos import Robot
from hpp.gepetto import Viewer
from tools.display_tools import *
import time
print "Plan guide trajectory ..."
import scenarios.memmo.talos_platform_random_path as tp
#Robot.urdfSuffix += "_safeFeet"
pId = 0


fullBody = Robot ()

# Set the bounds for the root
rootBounds = tp.rootBounds[::]
rootBounds[-2] -= 0.2
rootBounds[0] -= 0.2
rootBounds[1] += 0.2
rootBounds[2] -= 0.2
rootBounds[3] += 0.2
fullBody.setJointBounds ("root_joint",  rootBounds)
fullBody.setConstrainedJointsBounds()

# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
#load the viewer
try :
    v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)
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


# load a reference configuration
#q_ref = fullBody.referenceConfig[::]+[0]*6
q_ref = fullBody.referenceConfig_legsApart[::]+[0]*6
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)

fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)

fullBody.setCurrentConfig (q_init)

print "Generate limb DB ..."
tStart = time.time()
# generate databases : 

nbSamples = 100000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, "fixedStep06", 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.85)
#fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, "fixedStep06", 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.85)
#fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)


tGenerate =  time.time() - tStart
print "Done."
print "Databases generated in : "+str(tGenerate)+" s"

#define initial and final configurations : 
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0)[0:7] # use this to get the correct orientation
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
vel_init = tp.ps.configAtParam(pId,0)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0)[tp.indexECS+3:tp.indexECS+6]
vel_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = vel_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = vel_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]

'''
state_id = fullBody.generateStateInContact(q_init, vel_init,robustnessThreshold=5)
assert fullBody.rLegId in fullBody.getAllLimbsInContact(state_id), "Right leg not in contact in initial state"
assert fullBody.lLegId in fullBody.getAllLimbsInContact(state_id), "Left leg not in contact in initial state"
q_init = fullBody.getConfigAtState(state_id)
v(q_init)

state_id = fullBody.generateStateInContact(q_goal, vel_goal,robustnessThreshold=5)
assert fullBody.rLegId in fullBody.getAllLimbsInContact(state_id), "Right leg not in contact in final state"
assert fullBody.lLegId in fullBody.getAllLimbsInContact(state_id), "Left leg not in contact in final state"
q_goal = fullBody.getConfigAtState(state_id)
v(q_goal)
'''


fullBody.setStaticStability(True)
v.addLandmark('talos/base_link',0.3)

# FOR EASY SCENARIOS ?
q_init[2]=q_ref[2]
q_goal[2]=q_ref[2]


# define init gait according the direction of motion, try to move first the leg on the outside of the turn : 
if q_goal[0] > q_init[0] : #go toward x positif
  if q_goal[1] > q_init[1]: # turn left
    gait = [fullBody.rLegId,fullBody.lLegId]
  else : # turn right
    gait = [fullBody.lLegId,fullBody.rLegId]
else : # go toward x negatif
  if q_goal[1] > q_init[1]: # turn right
    gait = [fullBody.lLegId,fullBody.rLegId]
  else : # turn left
    gait = [fullBody.rLegId,fullBody.lLegId]

fullBody.setStartState(q_init,gait)
fullBody.setEndState(q_goal,gait)
v(q_init)
print "Generate contact plan ..."
tStart = time.time()
configs = fullBody.interpolate(0.005,pathId=pId,robustnessTreshold = 1, filterStates = True,testReachability=True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print "Done."
print "Contact plan generated in : "+str(tInterpolateConfigs)+" s"
print "number of configs :", len(configs)
#raw_input("Press Enter to display the contact sequence ...")
#displayContactSequence(v,configs)


# put back original bounds for wholebody methods
fullBody.resetJointsBounds()
displayContactSequence(v,configs)