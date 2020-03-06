from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
from hpp.corbaserver.rbprm.tools.display_tools import *
import flatGround_hrp2_pathKino as tp
import time
import numpy as np
tPlanning = tp.tPlanning


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
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)
r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)


q_init =[0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
qfar=q_ref[::]
qfar[2] = -5

#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
lLegId = 'hrp2_lleg_rom'
tStart = time.time()
fullBody.setReferenceConfig (q_ref)

rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "forward", 0.01,"_6_DOF",limbOffset=rLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullSize/RLEG_JOINT0_com_constraints.obj",kinematicConstraintsMin=0.3)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "forward", 0.01,"_6_DOF",kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/fullSize/LLEG_JOINT0_com_constraints.obj",limbOffset=lLegLimbOffset,kinematicConstraintsMin=0.3)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")
fullBody.setReferenceConfig (q_ref)
## Add arms (not used for contact) :


"""
rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
fullBody.addNonContactingLimb(rarmId,rarm,rHand, 10000)
fullBody.runLimbSampleAnalysis(rarmId, "ReferenceConfiguration", True)
larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
fullBody.addNonContactingLimb(larmId,larm,lHand, 10000)
fullBody.runLimbSampleAnalysis(larmId, "ReferenceConfiguration", True)
"""

"""
rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
fullBody.addLimb(rarmId,rarm,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=lLegLimbOffset,kinematicConstraintsMin=0.3)

fullBody.runLimbSampleAnalysis(rarmId, "ReferenceConfiguration", True)
larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
fullBody.addLimb(larmId,larm,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=lLegLimbOffset,kinematicConstraintsMin=0.3)

fullBody.runLimbSampleAnalysis(larmId, "ReferenceConfiguration", True)
"""



tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")


q_0 = fullBody.getCurrentConfig();
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)




configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(pId,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
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


# FIXME : test
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
r.addLandmark('hrp2_14/BODY',0.3)
r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])

"""
init_sid = fullBody.addState(q_init,[rLegId,lLegId])
int_sid = fullBody.addState(q_init,[rLegId])

fullBody.isReachableFromState(init_sid,int_sid)



displayOneStepConstraints(r)
"""


#p = fullBody.computeContactPointsAtState(init_sid)
#p = fullBody.computeContactPointsAtState(int_sid)


"""
q = q_init[::]
q[0] += 0.3
q = fullBody.generateContacts(q,dir_init,acc_init,robTreshold)
mid_sid = fullBody.addState(q,[lLegId,rLegId])
"""


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

tStart = time.time()
configsFull = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 0, filterStates = True,testReachability=False,quasiStatic=False)
tInterpolateConfigs = time.time() - tStart
print("number of configs :", len(configsFull))
r(configsFull[-1])

"""
import check_qp
planValid,curves_initGuess,timings_initGuess = check_qp.check_contact_plan(ps,r,pp,fullBody,0,len(configsFull))
print "Contact plan valid : "+str(planValid)
"""



from configs.straight_walk_dynamic_planning_config import *
from generate_contact_sequence import *

beginState = 0
endState = len(configsFull)-1
configs=configsFull[beginState:endState+1]
#cs = generateContactSequence(fullBody,configs,beginState, endState,r,curves_initGuess =curves_initGuess , timings_initGuess =timings_initGuess)
cs = generateContactSequence(fullBody,configs,beginState, endState,r)
#player.displayContactPlan()


filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print("save contact sequence : ",filename)



"""
f = open("/home/pfernbac/Documents/com_ineq_test/log_success.log","a")
f.write("num states : "+str(len(configsFull))+" \n")
f.close()
"""


"""

player = fullBodyPlayerHrp2.Player(fullBody,pp,tp,configsFull,draw=False,use_window=1,optim_effector=True,use_velocity=False,pathId = pId)




#player.displayContactPlan(1.)


r(fullBody.getConfigAtState(2))
q2 = fullBody.getConfigAtState(2)
q3 = fullBody.getConfigAtState(3)
q2[-6:]=[0]*6
q3[-6:]=[0]*6
r(q2)

s2 = fullBody.addState(q2,[rLegId,lLegId])
s3 = fullBody.addState(q3,[rLegId,lLegId])
pid = fullBody.isDynamicallyReachableFromState(2,3)
pid

displayBezierConstraints(r)

#r.client.gui.removeFromGroup("path_"+str(pid-1)+"_root",r.sceneName)
pp.displayPath(pid,r.color.blue)
createSphere("s",r)
moveSphere("s",r,x)
r(q_ref)


q1 = fullBody.getConfigAtState(2)
#q1[-3:]=[2,0,0]
r(q1)
s0 = fullBody.addState(q1,[rLegId,lLegId])
s1 = fullBody.addState(q1,[rLegId,lLegId])

fullBody.isReachableFromState(s0,s1)
displayOneStepConstraints(r)

#player.interpolate(2,len(configs)-1)

"""

""" # tests front line
s0 = State(fullBody,q=q_init,limbsIncontact = [rLegId,lLegId])
smid = State(fullBody,q=q_init,limbsIncontact = [lLegId])
sright = State(fullBody,q=q_init,limbsIncontact = [rLegId])
s02,success = StateHelper.addNewContact(s0,rLegId,[0.2,0-0.1,0.0],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s02.sId)
pIds = fullBody.isDynamicallyReachableFromState(s0.sId,s02.sId,True)
pIds = fullBody.isDynamicallyReachableFromState(s0.sId,s02.sId,numPointsPerPhases=0)



s04,success = StateHelper.addNewContact(s0,rLegId,[0.4,0-0.1,0.01],[0,0,1])
assert(success)
s05,success = StateHelper.addNewContact(s0,rLegId,[0.5,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s04.sId)
s06,success = StateHelper.addNewContact(s0,rLegId,[0.6,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s06.sId)
s07,success = StateHelper.addNewContact(s0,rLegId,[0.7,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s07.sId)
s075,success = StateHelper.addNewContact(s0,rLegId,[0.75,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s07.sId)
s08,success = StateHelper.addNewContact(s0,rLegId,[0.8,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s08.sId)

s085,success = StateHelper.addNewContact(s0,rLegId,[0.85,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s09.sId)

for i in range(1000):
  pIds = fullBody.isDynamicallyReachableFromState(s0.sId,s04.sId,numPointsPerPhases = 15)

for [s0,s1] in states:
  fullBody.isDynamicallyReachableFromState(s0.sId,s1.sId,numPointsPerPhases = 0)



for i in range(10000):
  q = fullBody.shootRandomConfig()
  s0 = State(fullBody,q=q,limbsIncontact = [lLegId])
  fullBody.isReachableFromState(s0.sId,s0.sId)



from parse_bench import *
parseBenchmark([])

from check_qp import check_traj_valid
check_traj_valid(ps,fullBody,s0,s02,pIds)


"""



"""#test backline

s0 = State(fullBody,q=q_init,limbsIncontact = [rLegId,lLegId])
s02,success = StateHelper.addNewContact(s0,rLegId,[-0.2,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s02.sId)
s04,success = StateHelper.addNewContact(s0,rLegId,[-0.4,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s04.sId)
s06,success = StateHelper.addNewContact(s0,rLegId,[-0.6,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s06.sId)
s07,success = StateHelper.addNewContact(s0,rLegId,[-0.7,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s07.sId)
s07i,success = StateHelper.removeContact(s0,rLegId)
fullBody.isReachableFromState(s0.sId,s07i.sId)
fullBody.isReachableFromState(s07i.sId,s07.sId)

s075,success = StateHelper.addNewContact(s0,rLegId,[-0.75,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s075.sId)
s075i,success = StateHelper.removeContact(s0,rLegId)
fullBody.isReachableFromState(s0.sId,s075i.sId)
fullBody.isReachableFromState(s075i.sId,s075.sId)

s08,success = StateHelper.addNewContact(s0,rLegId,[-0.8,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s08.sId)

s085,success = StateHelper.addNewContact(s0,rLegId,[-0.85,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s09.sId)
"""


# test right / diag
"""
s0 = State(fullBody,q=q_init,limbsIncontact = [rLegId,lLegId])
s02,success = StateHelper.addNewContact(s0,rLegId,[0.2,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s02.sId)
s04,success = StateHelper.addNewContact(s0,rLegId,[0.4,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s04.sId)
s06,success = StateHelper.addNewContact(s0,rLegId,[0.6,0-0.1,0.01],[0,0,1])
assert(success)
fullBody.isReachableFromState(s0.sId,s06.sId)
s07,success = StateHelper.addNewContact(s0,rLegId,[0.7,0-0.1,0.01],[0,0,1])
assert(success)
"""


