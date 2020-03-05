from talos_rbprm.talos import Robot
from hpp.gepetto import Viewer
import time
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import time


fullBody = Robot ()
fullBody.setJointBounds ("root_joint",  [-5,5, -1.5, 1.5, 0.95, 1.05])
fullBody.client.robot.setDimensionExtraConfigSpace(6)
fullBody.client.robot.setExtraConfigSpaceBounds([0]*12)
ps = ProblemSolver( fullBody )


from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)
vf.loadObstacleModel ("hpp_environments", "multicontact/table_140_70_73", "planning")



q_ref = [
        0.0, 0.0,  1.0232773,  0.0 ,  0.0, 0.0, 1,                   #Free flyer
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Left Leg
        0.0,  0.0, -0.411354,  0.859395, -0.448041, -0.001708,          #Right Leg
        0.0 ,  0.006761,                                                #Chest
        0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,-0.005,  #Left Arm
        -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,-0.005,#Right Arm
        0.,  0.,
0,0,0,0,0,0]; # head

q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)


tStart = time.time()
# generate databases :
nbSamples = 1
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/talos_rLeg_walk.db")
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.rArmId,fullBody.rarm,fullBody.rhand,fullBody.rArmOffset,fullBody.rArmNormal, fullBody.rArmx, fullBody.rArmx, nbSamples, "EFORT", 0.01)
fullBody.runLimbSampleAnalysis(fullBody.rArmId, "ReferenceConfiguration", True)

tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")

v = vf.createViewer(displayCoM=True)
v(q_init)
v.addLandmark(v.sceneName,0.5)
v.addLandmark('talos/arm_right_7_link',0.1)
q_init[0:2] = [-0.5,0.8]
q_init[32] -=1.5 # right elbow
v(q_init)

# sphere = target
from hpp.corbaserver.rbprm.tools.display_tools import *
createSphere('target',v,size=0.05,color=v.color.red)
v.client.gui.setVisibility('target','ON')
moveSphere('target',v,[0,0,0.5])

# create contact :
fullBody.setStartState(q_init,[fullBody.lLegId,fullBody.rLegId])
n = [0,0,1]

s0 = State(fullBody,q=q_init,limbsIncontact=[fullBody.lLegId,fullBody.rLegId])

###  move left foot of 30cm in front
pLLeg = s0.getCenterOfContactForLimb(fullBody.lLegId)[0]
pLLeg[0] += 0.3
pLLeg[2] -= 0.001 # required because this delta is added in rbprm ...
s1,success = StateHelper.addNewContact(s0,fullBody.lLegId,pLLeg,n,lockOtherJoints=False)
assert success, "Unable to project contact position for left foot"

###  move hand to the handle

pHand  = [0.1,0.5,0.75]
createSphere('s',v)
moveSphere('s',v,pHand)
s2,success = StateHelper.addNewContact(s1,fullBody.rArmId,pHand,n,lockOtherJoints=False)
assert success, "Unable to project contact position for right hand"
"""
v(s2.q())
com = fullBody.getCenterOfMass()
com[0] += 0.05
com[1] -= 0.1
com[2] -= 0.02
q3 = s2.projectToCOM(com,toNewState=True)
v(q3)
"""
"""
q1 = [-0.4179830939427893,
 0.8347627837183168,
 0.9946218381742752,
 -0.0867907317182463,
 0.08739711783836794,
 0.12321477028110761,
 0.9847066736170379,
 -0.2272949051024129,
 0.23532649004655146,
 -0.6159175763163939,
 1.0391593108867676,
 -0.5788741314607475,
 -0.04386597370357985,
 -0.22948450208879723,
 0.22171247100334404,
 -0.47112214269424046,
 0.8697762151341462,
 -0.5537904533973627,
 -0.030085935384065905,
 0.7648494558716157,
 -0.26096759550246706,
 0.3757914594070104,
 0.4250988458823696,
 0.04725271385732758,
 -0.44377218500455007,
 -8.056875771922206e-06,
 0.01562999972463939,
 0.11397873419634344,
 -0.0051879023889214865,
 -0.15886010249154855,
 -0.618674869330666,
 -0.5257508117875245,
 -0.19689267377445485,
 -0.6491975697318826,
 0.14357903922179352,
 0.698131700798,
 -0.005309093376355925,
 -0.015058725064333728,
 -0.0014257481207303427,
0,
0,
0,
0,
0,
0]

#s1 = State(fullBody,q=q1,limbsIncontact=[fullBody.lLegId,fullBody.rLegId,fullBody.rArmId])


q2 = [-0.1739726835171786,
 0.725403435762449,
 0.9672105544468215,
 -0.08712927323144357,
 0.1760957825181967,
 0.08020446105256104,
 0.9772236231041146,
 -0.049559661418543045,
 0.40455974164339564,
 -0.4003710685384243,
 0.7911412899690295,
 -0.7581686282462333,
 -0.21993634278507995,
 -0.055572016160718905,
 0.3901047358882102,
 -0.2328211479293735,
 0.5024200097788586,
 -0.6346706854454504,
 -0.20445467325620423,
 0.1867985328466371,
 0.785398163397,
 0.17358863127762683,
 0.0,
 -0.0418602806703596,
 -0.5374201900273251,
 0.0006980663134041395,
 -0.013186731949612358,
 0.09817522991123807,
 -0.005239088837162844,
 -0.5118702629493616,
 -0.12288822176799163,
 0.5082036732444083,
 -1.978470198798287,
 -0.28424506370808994,
 -0.1134585429671788,
 0.698131700798,
 -0.005036396255584366,
 0.008020752595769589,
 0.0008315553354996808,
0,
0,
0,
0,
0,
0]

s2 = State(fullBody,q=q2,limbsIncontact=[fullBody.lLegId,fullBody.rLegId,fullBody.rArmId])
v(s2.q())

com = fullBody.getCenterOfMass()

comF = [-0.24796187, 0.74366786,  0.81803711]
com=comF[::]
com[0] = -0.2165
moveSphere("com",v,com)
s2.projectToCOM(com)


s3 = State(fullBody,q=s2.q(),limbsIncontact=[fullBody.lLegId,fullBody.rLegId,fullBody.rArmId])
s3.projectToCOM(com,10000)
v(s3.q())

"""
configs = [s0.q(), s1.q(),s2.q()]
#configs = [s0.q(),s2.q()]

v(configs[-1])


