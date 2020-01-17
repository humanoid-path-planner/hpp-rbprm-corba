from hpp.corbaserver.rbprm.hyq_contact6D import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import time
print("Plan guide trajectory ...")
import hyq_slalom_debris_path as tp
print("Done.")
import time

pId = tp.ps.numberPaths() -1
fullBody = Robot ()

# Set the bounds for the root
root_bounds = tp.root_bounds
root_bounds[4] -= 0.05
root_bounds[5] += 0.05
fullBody.setJointBounds ("root_joint",  root_bounds)
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
#load the viewer
v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)

# load a reference configuration
q_ref = fullBody.referenceConfig[::]+[0]*6
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig (q_init)

print("Generate limb DB ...")
tStart = time.time()
# generate databases :

nbSamples = 10000

fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "fixedStep06", 0.01)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "fixedStep06", 0.01)
fullBody.addLimb(fullBody.rArmId,fullBody.rarm,fullBody.rhand,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "fixedStep06", 0.01)
fullBody.addLimb(fullBody.lArmId,fullBody.larm,fullBody.lhand,fullBody.offset,fullBody.normal, fullBody.legx, fullBody.legy, nbSamples, "fixedStep06", 0.01)

for limbId in fullBody.limbNames :
    fullBody.runLimbSampleAnalysis(limbId, "ReferenceConfiguration", True)


tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")

#define initial and final configurations :
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0.01)[0:7] # use this to get the correct orientation
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.01)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)

fullBody.setCurrentConfig (q_goal)

v.addLandmark('hyq/base_link',0.3)


q_init[2] = fullBody.referenceConfig[2]
q_goal[2] = fullBody.referenceConfig[2]
v(q_init)
#q_init = fullBody.generateContacts(q_init, [0,0,1])
#q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specify the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[fullBody.rLegId,fullBody.lArmId,fullBody.rArmId,fullBody.lLegId])
fullBody.setEndState(q_goal,[fullBody.rLegId,fullBody.lArmId,fullBody.rArmId,fullBody.lLegId])


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 2, filterStates = True,testReachability = False)
tInterpolateConfigs = time.time() - tStart
print("Done.")
print("Contact plan generated in : "+str(tInterpolateConfigs)+" s")
print("number of configs :", len(configs))
#raw_input("Press Enter to display the contact sequence ...")
#displayContactSequence(v,configs)



