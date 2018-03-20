from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
#from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
#from display_tools import *
import talos.flatGround_pyrene_pathKino as tp
import time

tPlanning = tp.tPlanning


packageName = "talos_description"
meshPackageName = "talos_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "talos"
urdfSuffix = "_full_v2"
srdfSuffix = ""
pId = tp.ps.numberPaths() -1
fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz",  [-5,5, -1.5, 1.5, 0.95, 1.05])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)
r = tp.Viewer (ps,viewerClient=tp.r.client, displayCoM = True)


q_ref = [
        0,0,1.0192720229567027,1,0,0,0, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0, # head
        0,0,0,0,0,0]; r (q_ref)

q_init = q_ref[::]

#fullBody.setReferenceConfig(q_ref)
"""
#test correspondance with reduced : 
q_init[19] = -0.5
q_init[20] = 0.8
r(q_init)
"""

fullBody.setCurrentConfig (q_init)
qfar=q_ref[::]
qfar[2] = -5

tStart = time.time()
nbSamples = 50000
rLegId = 'rleg'
rLeg = 'leg_right_1_joint'
rfoot = 'leg_right_sole_fix_joint'
rLegOffset = [0,0,0]
rLegNormal = [0,0,1]
rLegx = 0.1; rLegy = 0.06
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "forward", 0.01)
#fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)

lLegId = 'lleg'
lLeg = 'leg_left_1_joint'
lfoot = 'leg_left_sole_fix_joint'
lLegOffset = [0,0,0]
lLegNormal = [0,0,1]
lLegx = 0.1; lLegy = 0.06
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "forward", 0.01)
#fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)

"""
rarmId = 'rarm'
rarm = 'arm_right_1_joint'
rHand = 'gripper_right_inner_single_joint'
rArmOffset = [0,0,0.1]
rArmNormal = [0,0,1]
rArmx = 0.02; rArmy = 0.02
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(rarmId, "ReferenceConfiguration", True)

larmId = 'larm'
larm = 'arm_left_1_joint'
lHand = 'gripper_left_inner_single_joint'
lArmOffset = [0,0,-0.1]
lArmNormal = [0,0,1]
lArmx = 0.02; lArmy = 0.02
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(larmId, "ReferenceConfiguration", True)
"""



tGenerate =  time.time() - tStart
print "generate databases in : "+str(tGenerate)+" s"


q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)




configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

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

q_init[2] = q_ref[2]+0.01
q_goal[2] = q_ref[2]+0.01

fullBody.setStaticStability(True)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
r(q_init)
#q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
#q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('talos/base_link',0.3)
r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

tStart = time.time()
configsFull = fullBody.interpolate(0.001,pathId=pId,robustnessTreshold = 2, filterStates = False)
tInterpolateConfigs = time.time() - tStart
print "number of configs :", len(configsFull)




from planning.config import *
from generate_contact_sequence import *

beginState = 0
endState = len(configsFull)-2
configs=configsFull[beginState:endState+1]
cs = generateContactSequence(fullBody,configs,beginState, endState,r,curves_initGuess =curves_initGuess , timings_initGuess =timings_initGuess)
#player.displayContactPlan()


filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print "save contact sequence : ",filename





