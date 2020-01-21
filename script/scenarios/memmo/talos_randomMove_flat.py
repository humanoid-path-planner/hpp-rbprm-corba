from talos_rbprm.talos import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
from hpp.gepetto import ViewerFactory
from hpp.corbaserver import ProblemSolver
import os
import random
import time
print("Plan guide trajectory ...")
from . import talos_randomMove_path as tp
print("Done.")
import time
statusFilename = tp.statusFilename
pId = 0
f = open(statusFilename,"a")
if tp.ps.numberPaths() > 0 :
  print("Path planning OK.")
  f.write("Planning_success: True"+"\n")
  f.close()
else :
  print("Error during path planning")
  f.write("Planning_success: False"+"\n")
  f.close()
  import sys
  sys.exit(1)



fullBody = Robot ()
# Set the bounds for the root
fullBody.setJointBounds ("root_joint", [-0.3,0.3, -0.3, 0.3, tp.ROOT_Z_MIN, tp.ROOT_Z_MAX])
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
ps = tp.ProblemSolver( fullBody )
vf = ViewerFactory (ps)
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setRandomSeed(random.SystemRandom().randint(0, 999999))

#load the viewer
try :
    v = vf.createViewer(displayCoM = True)
except Exception:
    print("No viewer started !")
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


print("Generate limb DB ...")
tStart = time.time()
# generate databases :

nbSamples = 100000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, "fixedStep1", 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.7)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, "fixedStep1", 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.7)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)


tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")

## generate random initial state : root pose at the origin exepct for z translation and both feet in contact with the floor
from tools.sample_random_transition import sampleRandomStateFlatFloor
limbsInContact = [fullBody.rLegId,fullBody.lLegId]
random.seed()
floor_Z = -0.00095
floor_Z = 0.
s0 = sampleRandomStateFlatFloor(fullBody,limbsInContact,floor_Z)

q_init = s0.q()

q_goal = q_ref[::]
q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
q_goal[2] = q_ref[2]
robTreshold = 3

fullBody.setStaticStability(True)
v.addLandmark('talos/base_link',0.3)
v(q_init)

# specify the full body configurations as start and goal state of the problem

if q_goal[1] < 0: # goal on the right side of the circle, start motion with right leg first
  fullBody.setStartState(q_init,[fullBody.rLegId,fullBody.lLegId])
  fullBody.setEndState(q_goal,[fullBody.rLegId,fullBody.lLegId])
  print("Right foot first")
else :
  fullBody.setStartState(q_init,[fullBody.lLegId,fullBody.rLegId])
  fullBody.setEndState(q_goal,[fullBody.lLegId,fullBody.rLegId])
  print("Left foot first")

print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(0.005,pathId=pId,robustnessTreshold = robTreshold, filterStates = True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print("Done.")
print("Contact plan generated in : "+str(tInterpolateConfigs)+" s")
print("number of configs :", len(configs))
#raw_input("Press Enter to display the contact sequence ...")
#displayContactSequence(v,configs)


if len(configs) < 2 :
  cg_success = False
  print("Error during contact generation.")
else:
  cg_success = True
  print("Contact generation Done.")
if abs(configs[-1][0] - tp.q_goal[0]) < 0.01 and abs(configs[-1][1]- tp.q_goal[1]) < 0.01  and (len(fullBody.getContactsVariations(len(configs)-2,len(configs)-1))==1):
  print("Contact generation successful.")
  cg_reach_goal = True
else:
  print("Contact generation failed to reach the goal.")
  cg_reach_goal = False
if len(configs) > 10 :
  cg_too_many_states = True
  cg_success = False
  print("Discarded contact sequence because it was too long.")
else:
  cg_too_many_states = False

f = open(statusFilename,"a")
f.write("cg_success: "+str(cg_success)+"\n")
f.write("cg_reach_goal: "+str(cg_reach_goal)+"\n")
f.write("cg_too_many_states: "+str(cg_too_many_states)+"\n")
f.close()

if (not cg_success) or cg_too_many_states:
  import sys
  sys.exit(1)

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

