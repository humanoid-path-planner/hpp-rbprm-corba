from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
#from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
#from display_tools import *
import talos.airbus_no_plane_bigStairs_path as tp
import time
from planning.robot_config.talos import *
from display_tools import *

tPlanning = tp.tPlanning

##
#  Information to retrieve urdf and srdf files.

pId = tp.ps.numberPaths() -1
fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-13,10,-3.85,-3.75,-1.75,0.5])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)
r = tp.Viewer (ps,viewerClient=tp.r.client, displayCoM = True)


q_ref = [
        0.0, 0.0,  1.0232773,  1 ,  0.0, 0.0, 0.0,                   #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Right Leg
        0.0 ,  0.006761,                                                #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.  ,                                                       #Head
        0,0,0,0,0,0]; r (q_ref)
"""
# with arms in front
q_ref = [
        0.0, 0.0,  1.0232773,  1 ,  0.0, 0.0, 0.0,                   #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Right Leg
        0.0 ,  0.006761,                                                #Chest
        -0.5 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        0.5 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.  ,                                                       #Head
        0,0,0,0,0,0]; r (q_ref)
"""
q_init = q_ref[::]

fullBody.setReferenceConfig(q_ref)
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
# generate databases : 

nbSamples = 50000
rLegOffset = MRsole_offset.translation.transpose().tolist()[0]
rLegOffset[2] += 0.006
rLegNormal = [0,0,1]
rLegx = 0.1; rLegy = 0.06
fullBody.addLimb(rLegId,rleg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "fixedStep06", 0.01)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_rLeg_walk.db")

lLegOffset = MLsole_offset.translation.transpose().tolist()[0]
lLegOffset[2] += 0.006
lLegNormal = [0,0,1]
lLegx = 0.1; lLegy = 0.06
fullBody.addLimb(lLegId,lleg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "fixedStep06", 0.01)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_lLeg_walk.db")



rArmOffset = [0.055,-0.04,-0.13]
rArmNormal = [0,0,1]
rArmx = 0.005; rArmy = 0.005
fullBody.addLimb(rArmId,rarm,rhand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "manipulability", 0.01,"_6_DOF", False)
fullBody.runLimbSampleAnalysis(rArmId, "ReferenceConfiguration", True)

lArmOffset = [0.055,0.04,-0.13]
lArmNormal = [0,0,1]
lArmx = 0.005; lArmy = 0.005
fullBody.addLimb(lArmId,larm,lhand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "manipulability", 0.01,"_6_DOF", False)
fullBody.runLimbSampleAnalysis(lArmId, "ReferenceConfiguration", True)


"""
# load databases from files : 
fullBody.addLimbDatabase("./db/talos_rLeg_walk.db",rLegId,"fixedStep06")
fullBody.addLimbDatabase("./db/talos_lLeg_walk.db",lLegId,"fixedStep06")
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

robTreshold = 1
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


#q_init[2] = q_ref[2]
#q_goal[2] = q_ref[2]
q_init[2] = -1.57

fullBody.setStaticStability(False)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
r(q_init)
#q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('talos/base_link',0.3)
r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId,rArmId,lArmId])


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


tStart = time.time()
configsFull = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 0, filterStates = True,testReachability=False)
tInterpolateConfigs = time.time() - tStart
print "number of configs :", len(configsFull)



displayContactSequence(r,configsFull)

"""

from planning.configs.talos_flatGround import *
from generate_contact_sequence import *

beginState = 0
endState = len(configsFull)-2
configs=configsFull[beginState:endState+1]
cs = generateContactSequence(fullBody,configs,beginState, endState,r)
#player.displayContactPlan()


filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print "save contact sequence : ",filename

"""
createSphere('s',r,color=r.color.red)

q_init[2] = -1.57

s0 = State(fullBody,q=q_init,limbsIncontact = [rLegId,lLegId])
#s1,success = StateHelper.addNewContact(s0,rLegId,p,[0,0,1])
#assert(success)
#r.addLandmark('talos/arm_left_7_link',0.2)
p = [-12.3,-3.26,-1.2]

moveSphere('s',r,p)



s1 = s0

p = [-12.55,-3.26,-1.65]
s2,success = StateHelper.addNewContact(s1,lArmId,p,[0,0,1],10000)
r(s2.q())

p = [-12.55,-4.32,-1.65]
s3,success = StateHelper.addNewContact(s2,rArmId,p,[0,0,1],10000)
r(s3.q())

s3.projectToCOM(s0.getCenterOfMass(),10000)
r(s3.q())



p = [-12.5, -3.65, -2.39] # l foot first step
s4,success = StateHelper.addNewContact(s3,lLegId,p,[0,0,1],10000)
r(s4.q())





