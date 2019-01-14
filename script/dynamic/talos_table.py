from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import time


rLegId = 'talos_rleg_rom'
rLeg = 'leg_right_1_joint'
rFoot = 'leg_right_6_joint'

lLegId = 'talos_lleg_rom'
lLeg = 'leg_left_1_joint'
lFoot = 'leg_left_6_joint'

rArmId = 'talos_rarm_rom'
rArm = 'arm_right_1_joint'
rHand = 'arm_right_7_joint'

lArmId = 'talos_larm_rom'
lArm = 'arm_left_1_joint'
lHand = 'arm_left_7_joint'

packageName = "talos_data"
meshPackageName = "talos_data"
rootJointType = "freeflyer"    
urdfName = "talos"
urdfSuffix = "_reduced"
srdfSuffix = ""


fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("root_joint",  [-5,5, -1.5, 1.5, 0.95, 1.05])
ps = ProblemSolver( fullBody )


from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)
vf.loadObstacleModel ("hpp-rbprm-corba", "table_140_70_73", "planning")



q_ref = [
        0.0, 0.0,  1.0232773,  0.0 ,  0.0, 0.0, 1,                   #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Right Leg
        0.0 ,  0.006761,                                                #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.]; # head

q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)


tStart = time.time()
# generate databases : 

nbSamples = 1000
rLegOffset = [0,-0.00018,-0.107]
rLegOffset[2] += 0.006
rLegNormal = [0,0,1]
rLegx = 0.1; rLegy = 0.06
fullBody.addLimb(rLegId,rLeg,rFoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "static", 0.01)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_rLeg_walk.db")

lLegOffset = [0,-0.00018,-0.107]
lLegOffset[2] += 0.006
lLegNormal = [0,0,1]
lLegx = 0.1; lLegy = 0.06
fullBody.addLimb(lLegId,lLeg,lFoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "static", 0.01)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_lLeg_walk.db")


#rArmOffset = [0.055,-0.04,-0.13]
rArmOffset = [-0.01,0.,-0.154]
rArmNormal = [0,0,1]
rArmx = 0.005; rArmy = 0.005
fullBody.addLimb(rArmId,rArm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(rArmId, "ReferenceConfiguration", True)


"""
lArmOffset = [0.055,0.04,-0.13]
lArmNormal = [0,0,1]
lArmx = 0.02; lArmy = 0.02
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(larmId, "ReferenceConfiguration", True)
"""

tGenerate =  time.time() - tStart
print "generate databases in : "+str(tGenerate)+" s"

v = vf.createViewer(displayCoM=True)
v(q_init)
v.addLandmark(v.sceneName,0.5)
v.addLandmark('talos/arm_right_7_link',0.1)
q_init[0:2] = [-0.5,0.8]
q_init[32] -=1.5 # right elbow
v(q_init)

# sphere = target 
from display_tools import *
createSphere('target',v,size=0.05,color=v.color.red)
v.client.gui.setVisibility('target','ON')
moveSphere('target',v,[0,0,0.5])

# create contact : 
fullBody.setStartState(q_init,[lLegId,rLegId])
q_ref[0:3] = q_init[0:3]
sref = State(fullBody,q=q_ref,limbsIncontact=[lLegId,rLegId])
s0 = State(fullBody,q=q_init,limbsIncontact=[lLegId,rLegId])
createSphere('s',v)
p0 = [-0.25,0.5,0.75]
#p1 = [-0,0.45,0.8]
p = [0.1,0.5,0.75]
moveSphere('s',v,p)
s0_bis,success = StateHelper.addNewContact(sref,rArmId,p0,[0,0,1])
#s0_bis2,success = StateHelper.addNewContact(s0_bis,rArmId,p1,[0,0,1])
s1,success = StateHelper.addNewContact(s0_bis,rArmId,p,[0,0,1])
assert(success)
v(s1.q())

#project com
v(q_init)
com_i = fullBody.getCenterOfMass()
com_i[2] -= 0.03
com_i[0] += 0.06
createSphere("com",v)
moveSphere("com",v,com_i)
s1.projectToCOM(com_i)
v(s1.q())
s1_feet = State(fullBody,q=s1.q(),limbsIncontact=[lLegId,rLegId])

s2,success = StateHelper.addNewContact(s0_bis,rArmId,p,[0,0,1])
com=s2.getCenterOfMass()
#com[0] += 0.03
com[1] -= 0.06
com[2] -= 0.02
moveSphere("com",v,com)
s2.projectToCOM(com)
v(s2.q())

q3=q_init[::]
q3[20]=1.2
s3_0 = State(fullBody,q=q3,limbsIncontact=[lLegId,rLegId])
s3,success = StateHelper.addNewContact(s3_0,rArmId,p,[0,0,1])
assert(success)
com=s3.getCenterOfMass()
#com[0] += 0.03
com[1] -= 0.1
com[2] -= 0.02
moveSphere("com",v,com)
s3.projectToCOM(com)
v(s3.q())

"""
jointsName = [rFoot,lFoot,rHand]
contactPos = []
contactNormal = []
pn = s1.getCenterOfContactForLimb(rLegId)
contactPos += [pn[0]]
contactNormal += [pn[1]]
pn = s1.getCenterOfContactForLimb(lLegId)
contactPos += [pn[0]]
contactNormal += [pn[1]]
pn = s1.getCenterOfContactForLimb(rArmId)
contactPos += [pn[0]]
contactNormal += [pn[1]]


ps.client.problem.createStaticStabilityConstraint ('staticStability',
     jointsName, contactPos, contactNormal,'root_joint')
"""


q2 = q_ref[::]



