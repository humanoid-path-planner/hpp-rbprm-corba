from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import omniORB.any
import stair_bauzil_hrp2_path as tp
import time



packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced_safe"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz",  [0,2, -1, 1, 0, 2.2])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)

ps = tp.ProblemSolver( fullBody )


ps.client.problem.setParameter("aMax",omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(tp.vMax))
ps.client.problem.setParameter("friction",tp.mu)

r = tp.Viewer (ps,viewerClient=tp.r.client, displayCoM = True)
tStart = time.time()
#~ AFTER loading obstacles


#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
lLegId = 'hrp2_lleg_rom'
tStart = time.time()


rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.0955]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05

fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=rLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullSize/RLEG_JOINT0_com_constraints.obj",kinematicConstraintsMin=0.)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.0955]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=lLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullSize/LLEG_JOINT0_com_constraints.obj",kinematicConstraintsMin=0.)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")


rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,0.1]
rArmNormal = [0,0,-1]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 100000, "manipulability", 0.01, "_6_DOF", True)



tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")



q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)


q_init =[0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
fullBody.setReferenceConfig (q_ref)

configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:3] = tp.ps.configAtParam(0,0.01)[0:3] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:3] = tp.ps.configAtParam(0,tp.ps.pathLength(0))[0:3]
dir_init = tp.ps.configAtParam(0,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(0,0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[tp.indexECS:tp.indexECS+3]
acc_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[tp.indexECS+3:tp.indexECS+6]



fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)


# FIXME : test
#q_init[2] = q_init[2]+0.0
#q_goal[2] = q_goal[2]+0.02
q_init[2] = q_ref[2] + 0.01
q_goal[2] = 1.25




fullBody.setStaticStability(True)
"""
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
r(q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init)
r(q_init)
"""

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal)

# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]
# specifying the full body configurations as start and goal state of the problem

r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId])


tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=0,robustnessTreshold = 0, filterStates = True,testReachability=False,quasiStatic=False)
tInterpolateConfigs = time.time()-tStart
print("number of configs :", len(configs))
r(configs[-1])


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from fullBodyPlayerHrp2 import Player
player = Player(fullBody,pp,tp,configs,draw=False,optim_effector=False,use_velocity=False,pathId = 0)



player.displayContactPlan()


beginState=0
endState=len(configs)-1


from configs.stairs_config import *
from generate_contact_sequence import *
cs = generateContactSequence(fullBody,configs,beginState,endState,r)
filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print("save contact sequence : ",filename)



