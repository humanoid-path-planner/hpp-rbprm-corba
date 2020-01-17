from hpp.corbaserver.rbprm.anymal_contact6D import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import time
print("Plan guide trajectory ...")
import darpa_path as tp
print("Done.")
import time



pId = tp.ps.numberPaths() -1
fullBody = Robot ()
# Set the bounds for the root, take slightly larger bounding box than during root planning
root_bounds = tp.root_bounds[::]
root_bounds[0] -= 0.2
root_bounds[1] += 0.2
root_bounds[2] -= 0.2
root_bounds[3] += 0.2
root_bounds[-1] = 0.9
root_bounds[-2] = 0.4
fullBody.setJointBounds ("root_joint",  root_bounds)
# constraint the joints limits in a conservative manner.
# This is a 'hack' to help produce contact sequence requiring less torque
fullBody.setJointBounds("LF_KFE",[-1.5,0.])
fullBody.setJointBounds("RF_KFE",[-1.5,0.])
fullBody.setJointBounds("LH_KFE",[0.,1.5])
fullBody.setJointBounds("RH_KFE",[0.,1.5])
fullBody.setJointBounds("LH_HFE",[-2.35,0.])
fullBody.setJointBounds("RH_HFE",[-2.3,-0.1])
fullBody.setJointBounds("LF_HFE",[0.15,2.35])
fullBody.setJointBounds("RF_HFE",[0.3,2.35])
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,-tp.aMaxZ,tp.aMaxZ])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
#load the viewer
v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)

# load a reference configuration
q_ref = fullBody.referenceConfig[::]+[0,0,0,0,0,0]
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig (q_init)

dict_heuristic = {fullBody.rLegId:"static", fullBody.lLegId:"static", fullBody.rArmId:"fixedStep04", fullBody.lArmId:"fixedStep04"}
print("Generate limb DB ...")
tStart = time.time()
fullBody.loadAllLimbs(dict_heuristic,"ReferenceConfigurationCustom",nbSamples=50000)
tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")
fullBody.setReferenceConfig(q_ref)

#define initial and final configurations :
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0.01)[0:7] # use this to get the correct orientation
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.01)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 0
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

v.addLandmark('anymal_reachability/base',0.3)
v(q_init)

# specify the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,fullBody.limbs_names)
fullBody.setEndState(q_goal,fullBody.limbs_names)


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = robTreshold, filterStates = True,testReachability=True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print("Done. ")
print("Contact sequence computed in "+str(tInterpolateConfigs)+" s.")
print("number of configs :", len(configs))
input("Press Enter to display the contact sequence ...")
displayContactSequence(v,configs)

import tools.createActionDRP as exp


