from hpp.corbaserver.rbprm.talos import Robot
from hpp.gepetto import Viewer
from tools.display_tools import *
import time
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
print "Plan guide trajectory ..."
import talos_navBauzil_path as tp
print "Done."
import time



pId = tp.ps.numberPaths() -1
fullBody = Robot ()

root_bounds = tp.root_bounds[::]
root_bounds[0] -= 0.2
root_bounds[1] += 0.2
root_bounds[2] -= 0.2
root_bounds[3] += 0.2
root_bounds[-1] = 1.2
root_bounds[-2] = 0.8
fullBody.setJointBounds ("root_joint",  root_bounds)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)



q_ref = fullBody.referenceConfig[::]+[0,0,0,0,0,0]
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig (q_init)


print "Generate limb DB ..."
tStart = time.time()
# generate databases : 

nbSamples = 10000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, "fixedStep08", 0.01)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_rLeg_walk.db")

fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, "fixedStep08", 0.01)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_lLeg_walk.db")



tGenerate =  time.time() - tStart
print "Done."
print "Databases generated in : "+str(tGenerate)+" s"


q_0 = fullBody.getCurrentConfig(); 
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


q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]


fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
v(q_init)

fullBody.setCurrentConfig (q_goal)
v(q_goal)

# specifying the full body configurations as start and goal state of the problem
v.addLandmark('talos/base_link',0.3)
v(q_init)

fullBody.setStartState(q_init,[fullBody.lLegId,fullBody.rLegId])
fullBody.setEndState(q_goal,[fullBody.lLegId,fullBody.rLegId])


from hpp.gepetto import PathPlayer
pp = PathPlayer ( v)

print "Generate contact plan ..."
tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 2, filterStates = True,testReachability=False)
tInterpolateConfigs = time.time() - tStart
print "Done."
print "number of configs :", len(configs)
raw_input("Press Enter to display the contact sequence ...")
displayContactSequence(v,configs)



