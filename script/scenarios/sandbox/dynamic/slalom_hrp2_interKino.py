from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from tools import *
import slalom_hrp2_pathKino as tp
import time
import omniORB.any


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
fullBody.setJointBounds ("base_joint_xyz",  [-5.5,5.5, -2.5, 2.5, 0.55, 0.65])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)

ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(tp.vMax))

r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)

q_init =[0, 0, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
fullBody.setReferenceConfig (q_ref)



#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
lLegId = 'hrp2_lleg_rom'
tStart = time.time()


rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegLimbOffset=[0,0,-0.06]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "forward", 0.1,"_6_DOF")
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "dynamicWalk", 0.05,"_6_DOF",limbOffset=rLegLimbOffset)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegLimbOffset=[0,0,0.06]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "forward", 0.1,"_6_DOF")
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "dynamicWalk", 0.05,"_6_DOF",limbOffset=lLegLimbOffset)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")


tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")


"""
fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
tLoad =  time.time() - tStart
print "Load databases in : "+str(tLoad)+" s"
"""


q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)




configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(pId,0)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 1
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


# FIXME : test
q_init[2] = q_init[2]+0.1
q_goal[2] = q_goal[2]+0.1


# Randomly generating a contact configuration at q_init
fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
r(q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('hrp2_14/BODY',0.3)
r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])
fullBody.setStaticStability(False) # only set it after the init/goal configuration are computed



from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = robTreshold, filterStates = True)
tInterpolate = time.time()-tStart
print("number of configs : ", len(configs))
print("generated in "+str(tInterpolate)+" s")
r(configs[len(configs)-2])


player = fullBodyPlayerHrp2.Player(fullBody,pp,tp,configs,draw=False,use_window=1,optim_effector=True,use_velocity=False,pathId = pId)

# remove the last config (= user defined q_goal, not consitent with the previous state)

#r(configs[0])
#player.displayContactPlan(1.)

#player.interpolate(2,len(configs)-1)

"""
r(configs[5])
dir = configs[5][37:40]
fullBody.client.rbprm.rbprm.evaluateConfig(configs[5],dir)
"""

from planning.config import *
from generate_contact_sequence import *
cs = generateContactSequence(fullBody,configs,r)
filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print("save contact sequence : ",filename)


r(q_init)
pos=fullBody.getJointPosition('RLEG_JOINT0')
addSphere(r,r.color.blue,pos)



id = r.client.gui.getWindowID("window_hpp_")
r.client.gui.attachCameraToNode( 'hrp2_14/BODY_0',id)



