from hpp.corbaserver.rbprm.talos import Robot
from hpp.gepetto import Viewer
from tools.display_tools import *
from hpp.gepetto import ViewerFactory
from hpp.corbaserver import ProblemSolver
import os
import random
import time
statusFilename = "/res/infos.log"



fullBody = Robot ()
# Set the bounds for the root
fullBody.setJointBounds ("root_joint", [-0.3,0.3, -0.3, 0.3, 1.01, 1.03])
## reduce bounds on joints along x, to put conservative condition on the contact generation for sideway steps
joint6L_bounds_prev=fullBody.getJointBounds('leg_left_6_joint')
joint2L_bounds_prev=fullBody.getJointBounds('leg_left_2_joint')
joint6R_bounds_prev=fullBody.getJointBounds('leg_right_6_joint')
joint2R_bounds_prev=fullBody.getJointBounds('leg_right_2_joint')
fullBody.setJointBounds('leg_left_6_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_left_2_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_right_6_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_right_2_joint',[-0.25,0.25])
# constraint z axis and y axis : 
joint1L_bounds_prev=fullBody.getJointBounds('leg_left_1_joint')
joint3L_bounds_prev=fullBody.getJointBounds('leg_left_3_joint')
joint1R_bounds_prev=fullBody.getJointBounds('leg_right_1_joint')
joint3R_bounds_prev=fullBody.getJointBounds('leg_right_3_joint')
fullBody.setJointBounds('leg_left_1_joint',[-0.2,0.7])
fullBody.setJointBounds('leg_left_3_joint',[-1.3,0.4])
fullBody.setJointBounds('leg_right_1_joint',[-0.7,0.2])
fullBody.setJointBounds('leg_right_3_joint',[-1.3,0.4])

# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(6)
vMax = 0.5# linear velocity bound for the root
aMax = 0.5# linear acceleration bound for the root
fullBody.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
ps = ProblemSolver( fullBody )
vf = ViewerFactory (ps)
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setRandomSeed(random.SystemRandom().randint(0, 999999))

#load the viewer
try :
    v = vf.createViewer(displayCoM = True)
except Exception:
    print "No viewer started !"
    class FakeViewer():
        sceneName = ""
        def __init__(self):
            return
        def __call__(self,q):
            return
        def addLandmark(self,a,b):
            return
    v = FakeViewer()

v.addLandmark(v.sceneName,0.5)
v.addLandmark('talos/base_link',0.3)

# load a reference configuration
q_ref = fullBody.referenceConfig[::]+[0]*6
#q_ref = fullBody.referenceConfig_legsApart[::]+[0]*6
fullBody.setReferenceConfig(q_ref)
fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)


print "Generate limb DB ..."
tStart = time.time()
# generate databases : 

nbSamples = 10
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, "static", 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.7)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, "static", 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.7)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)


tGenerate =  time.time() - tStart
print "Done."
print "Databases generated in : "+str(tGenerate)+" s"

from tools.sample_random_transition import sampleRandomTransitionFlatFloor
limbsInContact = [fullBody.rLegId,fullBody.lLegId]
random.seed()
movingId = random.randint(0,1)
movingLimb = limbsInContact[movingId]
print "Move limb : ",movingLimb
#floor_Z = - fullBody.dict_offset['leg_left_6_joint'].translation[2,0]
floor_Z = 0.
s0,s1 = sampleRandomTransitionFlatFloor(fullBody,limbsInContact,movingLimb,floor_Z)

configs = [s0.q(),s1.q()]
beginId = s0.sId
endId = beginId + 1
v(configs[0])

if len(configs) == 2 :
    cg_success = True
    cg_reach_goal = True
    print "Contact generation successful."
else:
    cg_success = False
    cg_reach_goal = False
    print "Contact generation failed."


f = open(statusFilename,"w")
f.write("q_init= "+str(s0.q())+"\n")
f.write("q_goal= "+str(s1.q())+"\n")
f.write("cg_success: "+str(cg_success)+"\n")
f.write("cg_reach_goal: "+str(cg_reach_goal)+"\n")
f.close()


# put back original bounds for wholebody methods
fullBody.setJointBounds ("root_joint", [-2,2, -2, 2, 0.6, 1.4])
fullBody.setJointBounds('leg_left_6_joint',joint6L_bounds_prev)
fullBody.setJointBounds('leg_left_2_joint',joint2L_bounds_prev)
fullBody.setJointBounds('leg_right_6_joint',joint6R_bounds_prev)
fullBody.setJointBounds('leg_right_2_joint',joint2R_bounds_prev)
fullBody.setJointBounds('leg_left_1_joint',joint1L_bounds_prev)
fullBody.setJointBounds('leg_left_3_joint',joint3L_bounds_prev)
fullBody.setJointBounds('leg_right_1_joint',joint1R_bounds_prev)
fullBody.setJointBounds('leg_right_3_joint',joint3R_bounds_prev)

