from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from display_tools import *
import time
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
print "Plan guide trajectory ..."
import talos_flatGround_path as tp
print "Done."
import time

##
#  Information to retrieve urdf and srdf files.
packageName = "talos_data"
meshPackageName = "talos_data"
rootJointType = "freeflyer"    
urdfName = "talos"
urdfSuffix = "_reduced"
srdfSuffix = ""


rLegId = 'talos_rleg_rom'
rleg = 'leg_right_1_joint'
rfoot = 'leg_right_6_joint'

lLegId = 'talos_lleg_rom'
lleg = 'leg_left_1_joint'
lfoot = 'leg_left_6_joint'

rArmId = 'talos_rarm_rom'
rarm = 'arm_right_1_joint'
rhand = 'arm_right_7_joint'

lArmId = 'talos_larm_rom'
larm = 'arm_left_1_joint'
lhand = 'arm_left_7_joint'


pId = tp.ps.numberPaths() -1
fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("root_joint",  [-5,5, -1.5, 1.5, 0.95, 1.05])
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)


q_ref = [
        0.0, 0.0,  1.0232773,  0.0 ,  0.0, 0.0, 1.,                   #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Right Leg
        0.0 ,  0.006761,                                                #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.  ,                                                       #Head
        0,0,0,0,0,0]; v (q_ref)

q_init = q_ref[::]

fullBody.setReferenceConfig(q_ref)

fullBody.setCurrentConfig (q_init)

print "Generate limb DB ..."
tStart = time.time()
# generate databases : 

nbSamples = 10000
rLegOffset = [0.,  -0.00018, -0.107]
rLegOffset[2] += 0.006
rLegNormal = [0,0,1]
rLegx = 0.1; rLegy = 0.06
fullBody.addLimb(rLegId,rleg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "fixedStep08", 0.01)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_rLeg_walk.db")

lLegOffset = [0.,  -0.00018, -0.107]
lLegOffset[2] += 0.006
lLegNormal = [0,0,1]
lLegx = 0.1; lLegy = 0.06
fullBody.addLimb(lLegId,lleg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "fixedStep08", 0.01)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
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

fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])


from hpp.gepetto import PathPlayer
pp = PathPlayer ( v)

print "Generate contact plan ..."
tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 2, filterStates = False)
tInterpolateConfigs = time.time() - tStart
print "Done."
print "number of configs :", len(configs)
raw_input("Press Enter to display the contact sequence ...")
displayContactSequence(v,configs)



