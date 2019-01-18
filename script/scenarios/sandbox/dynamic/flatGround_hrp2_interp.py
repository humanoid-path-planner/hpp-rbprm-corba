from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

print "####################################"
print "#            SOLVING P1 :          #"
import flatGround_hrp2_pathKino as tp
print "#               DONE               #"
print "####################################"
print "#            SOLVING P2 :          #"
print "####################################"
import time



packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""
pId = tp.ps.numberPaths() -1
fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-5,5, -1.5, 1.5, 0.5, 0.8])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)

ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)
ps.client.problem.setParameter("friction",tp.mu)
r = tp.Viewer (ps,viewerClient=tp.r.client)

#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "forward", 0.1)

lLegId = 'hrp2_lleg_rom'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "forward", 0.1)




q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)


q_init =[0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
fullBody.setCurrentConfig (q_init)

configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(pId,0.0001)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.0001)[0:7]
dir_init = tp.ps.configAtParam(pId,0.0001)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0.0001)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.0001)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 2
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


# FIXME : test
q_init[2] = q_init[2]+0.1
q_goal[2] = q_goal[2]+0.1

fullBody.setStaticStability(False)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
r(q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem

r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])





fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 1, filterStates = True)
print "number of configs :", len(configs)




player = fullBodyPlayerHrp2.Player(fullBody,pp,tp,configs,draw=False,use_window=1,optim_effector=True,use_velocity=False,pathId = pId)

configs = configs[:-1]

# remove the last config (= user defined q_goal, not consitent with the previous state)


player.displayContactPlan(1.)

#player.interpolate(2,len(configs)-1)
print "####################################"
print "#            SOLVING P2 :          #"
print "#               DONE               #"
print "####################################"
print "# Writing contact sequence file :  #"
print "####################################"

from planning.configs.straight_walk_dynamic_config import *
from generate_contact_sequence import *
cs = generateContactSequence(fullBody,configs,r)
filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, CONTACT_SEQUENCE_XML_TAG)
print "save contact sequence : ",filename
print "####################################"
print "# Writing contact sequence file :  #"
print "#               DONE               #"
print "####################################"


