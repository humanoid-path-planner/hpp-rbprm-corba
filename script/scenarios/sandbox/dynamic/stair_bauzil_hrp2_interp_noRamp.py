from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import omniORB.any
from . import stair_bauzil_hrp2_path_noRamp as tp
import time
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper

from hpp.corbaserver.rbprm.tools.display_tools import *
from constraint_to_dae import *


packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz",  [0,1.55, -0.25, -0.15, 0.2, 2])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )


ps.client.problem.setParameter("aMax",omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("aMaxZ",omniORB.any.to_any(1.))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(tp.vMax))
ps.client.problem.setParameter("friction",tp.mu)


r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)

#~ AFTER loading obstacles


lLegId = 'hrp2_lleg_rom'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,lLegNormal, lLegx, lLegy, 50000, "manipulability", 0.01,"_6_DOF")


"""
rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1]
rArmNormal = [0,0,1]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 100000, "manipulability", 0.01, "_6_DOF", True)

"""

rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "manipulability", 0.01,"_6_DOF")


"""

#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, 0.05)

rKneeId = '0RKnee'
rLeg = 'RLEG_JOINT0'
rKnee = 'RLEG_JOINT3'
rLegOffset = [0.105,0.055,0.017]
rLegNormal = [-1,0,0]
rLegx = 0.05; rLegy = 0.05
#~ fullBody.addLimb(rKneeId, rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 10000, 0.01)
#~
lKneeId = '1LKnee'
lLeg = 'LLEG_JOINT0'
lKnee = 'LLEG_JOINT3'
lLegOffset = [0.105,0.055,0.017]
lLegNormal = [-1,0,0]
lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lKneeId,lLeg,lKnee,lLegOffset,lLegNormal, lLegx, lLegy, 10000, 0.01)
 #~

#~ fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
#~ fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])


"""


q_0 = fullBody.getCurrentConfig();
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)


q_init =[0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
fullBody.setReferenceConfig (q_ref)

configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(0,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(0,tp.ps.pathLength(0))[0:7]
dir_init = tp.ps.configAtParam(0,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(0,0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[tp.indexECS:tp.indexECS+3]
acc_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[tp.indexECS+3:tp.indexECS+6]



fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)


# FIXME : test
q_init[2] = q_init[2]+0.02
q_goal[2] = q_goal[2]+0.02

#q_init[0:3]=[0.28994563306701016,-0.82,0.6191688248477717]

fullBody.setStaticStability(True)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init) ; r(q_init)


# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)


# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]
# specifying the full body configurations as start and goal state of the problem
q_init = fullBody.generateContacts(q_init, dir_init,acc_init,1)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,1)

r(q_init)

fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)
pp.dt=0.001

configs = fullBody.interpolate(0.05,pathId=0,robustnessTreshold = 0, filterStates = True)
print("number of configs :", len(configs))
r(configs[-1])



noCOQP = 0

for i in range(len(configs)-1):
  pid = fullBody.isDynamicallyReachableFromState(i,i+1)
  if len(pid)==0:
    noCOQP +=1




f = open("/local/fernbac/bench_iros18/success/log_successStairsNoRamp.log","a")
if noCOQP>0:
  f.write("fail, with "+str(noCOQP)+" infeasibles transitions\n")
else:
  f.write("all transition feasibles\n")
f.close()





"""

from fullBodyPlayerHrp2 import Player
player = Player(fullBody,pp,tp,configs,draw=False,optim_effector=False,use_velocity=False,pathId = 0)



player.displayContactPlan()
"""


"""
from check_qp import *
check_one_transition(ps,fullBody,s0,s2)
check_contact_planps,r,pp,fullBody,0,len(configs)-1)

from planning.config import *
from generate_contact_sequence import *

beginState = 0
endState = len(configs)-1

cs = generateContactSequence(fullBody,configs,beginState, endState,r)


filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print "save contact sequence : ",filename

"""

"""



from constraint_to_dae import *


fullBody.isReachableFromState(3,4)
displayTwoStepConstraints(r,True)


removeAllConstraints(r)

s_m = fullBody.addState(fullBody.getConfigAtState(1),[rLegId,rarmId])

fullBody.isReachableFromState(1,s_m)
displayOneStepConstraints(r)


s_m = fullBody.addState(fullBody.getConfigAtState(7),[rLegId])

fullBody.isReachableFromState(7,s_m)
displayOneStepConstraints(r)

print "####################################"
print "#            SOLVING P2 :          #"
print "#               DONE               #"
print "####################################"
print "# Writing contact sequence file :  #"
print "####################################"

from planning.configs.stairs_config import *
from generate_contact_sequence import *
cs = generateContactSequence(fullBody,configs[:5],r)
filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, CONTACT_SEQUENCE_XML_TAG)
print "save contact sequence : ",filename
print "####################################"
print "# Writing contact sequence file :  #"
print "#               DONE               #"
print "####################################"


"""

