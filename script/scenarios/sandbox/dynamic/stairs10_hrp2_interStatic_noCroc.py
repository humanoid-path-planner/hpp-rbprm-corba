from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from tools import *
import stairs10_hrp2_pathKino as tp
import time
import omniORB.any
from constraint_to_dae import *
from hpp.corbaserver.rbprm.tools.display_tools import *
from configs.stairs10_bauzil_stairs import *
from disp_bezier import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
#urdfSuffix = "_reduced_safe20"
urdfSuffix = "_reduced"
srdfSuffix = ""
pId = tp.ps.numberPaths() -1
fullBody = FullBody ()
tPlanning = 0.

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz",  [0,2,0.6 ,1.1, 0.35, 1.5])
fullBody.setJointBounds ("LLEG_JOINT3",  [0.3,2.61799])
fullBody.setJointBounds ("RLEG_JOINT3", [0.3,2.61799])


fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(tp.vMax))

r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = False, displayCoM = True)

view = [0.8,
 0.75,
 4.85,
 -0.7071,
 0,
 0,
 0.7071]

r.client.gui.setCameraTransform(0,view)



q_init =[0, 0, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)

fullBody.setReferenceConfig (q_init)


#~ AFTER loading obstacles
tStart = time.time()



rLegOffset = [0,0,-0.0955]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(rLegId,rleg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "forward", 0.01,"_6_DOF",limbOffset=rLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullsize/RLEG_JOINT0_com_constraints.obj",kinematicConstraintsMin=0.3)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")


lLegOffset = [0,0,-0.0955]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(lLegId,lleg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "forward", 0.01,"_6_DOF",limbOffset=lLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullsize/LLEG_JOINT0_com_constraints.obj",kinematicConstraintsMin=0.3)
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




eps=0.0001
configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,eps)[0:7] # use this to get the correct orientation
q_goal = q_ref[::]
q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.0001)[0:7]
dir_init = [0,0,0]
acc_init = [0,0,0]
dir_goal = tp.ps.configAtParam(pId,eps)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


# FIXME : test
q_init[2] = q_ref[2] + 0.0005
q_goal[2] = q_ref[2] +0.3005

# Randomly generating a contact configuration at q_init
fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
r(q_init)


# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
#q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('hrp2_14/BODY',0.3)
r(q_init)


fullBody.setStartState(q_init,[lLegId,rLegId])
fullBody.setEndState(q_goal,[lLegId,rLegId])
fullBody.setStaticStability(True) # only set it after the init/goal configuration are computed



from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

tStart = time.time()
configsFull = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = robTreshold, filterStates = True,testReachability=False,quasiStatic=False)
tInterpolateConfigs = time.time()-tStart

print("number of configs : ", len(configsFull))
print("generated in "+str(tInterpolateConfigs)+" s")
r(configsFull[len(configsFull)-1])





from generate_contact_sequence import *

beginState = 0
endState = len(configsFull)-1
configs=configsFull[beginState:endState+1]
cs = generateContactSequence(fullBody,configs,beginState, endState,r)

#displayContactSequence(r,configsFull)


filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print("save contact sequence : ",filename)

"""

s0 = State(fullBody,q=q_init,limbsIncontact = [rLegId,lLegId])
s0 = State(fullBody,q=configsFull[10],limbsIncontact = [rLegId,lLegId])
s1,success = StateHelper.removeContact(s0,lLegId)
r(s1.q())
fullBody.isReachableFromState(s1.sId,s1.sId)
displayOneStepConstraints(r)


"""



