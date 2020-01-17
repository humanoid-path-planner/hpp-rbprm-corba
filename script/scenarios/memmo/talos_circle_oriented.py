from talos_rbprm.talos import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import numpy as np
import time
print("Plan guide trajectory ...")
import talos_circle_oriented_path as tp
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
fullBody.setJointBounds ("root_joint",  [-2,2, -2, 2, 0.9, 1.05])
## reduce bounds on joints along x, to put conservative condition on the contact generation for sideway steps
joint1L_bounds_prev=fullBody.getJointBounds('leg_left_1_joint')
joint6L_bounds_prev=fullBody.getJointBounds('leg_left_6_joint')
joint2L_bounds_prev=fullBody.getJointBounds('leg_left_2_joint')
joint1R_bounds_prev=fullBody.getJointBounds('leg_right_1_joint')
joint6R_bounds_prev=fullBody.getJointBounds('leg_right_6_joint')
joint2R_bounds_prev=fullBody.getJointBounds('leg_right_2_joint')
fullBody.setJointBounds('leg_left_1_joint',[-0.34,1.4])
fullBody.setJointBounds('leg_left_6_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_left_2_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_right_1_joint',[-1.4,0.34])
fullBody.setJointBounds('leg_right_6_joint',[-0.25,0.25])
fullBody.setJointBounds('leg_right_2_joint',[-0.25,0.25])
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
#load the viewer
try :
    v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)
except Exception:
    print("No viewer started !")
    class FakeViewer():
        def __init__(self):
            return
        def __call__(self,q):
            return
        def addLandmark(self,a,b):
            return
    v = FakeViewer()


# load a reference configuration
q_ref = fullBody.referenceConfig[::]+[0]*6
#q_ref = fullBody.referenceConfig_legsApart[::]+[0]*6
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)


fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)
if tp.alpha <0.8*np.pi and tp.alpha > 0.2*np.pi : # nearly straight walk
    heuristicR = "fixedStep08"
    heuristicL = "fixedStep08"
    fullBody.usePosturalTaskContactCreation(True)
else: # more complex motion. Do smaller steps and allow non-straight feet orientation
    heuristicR = "fixedStep06"
    heuristicL = "fixedStep06"

fullBody.setCurrentConfig (q_init)

print("Generate limb DB ...")
tStart = time.time()
# generate databases :

nbSamples = 100000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, heuristicR, 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, heuristicL, 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.85)


if not(tp.alpha <0.8*np.pi and tp.alpha > 0.2*np.pi) : # not close to straight line
    fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
    fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)

tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")

#define initial and final configurations :
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0)[0:7]
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
vel_init = [0,0,0]
acc_init = [0,0,0]
vel_goal = [0,0,0]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = vel_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = vel_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]


q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]


fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
v(q_init)

fullBody.setCurrentConfig (q_goal)
v(q_goal)

v.addLandmark('talos/base_link',0.3)
v(q_init)
#fullBody.setReferenceConfig(fullBody.referenceConfig_legsApart[::]+[0]*6)


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
configs = fullBody.interpolate(0.005,pathId=pId,robustnessTreshold = 2, filterStates = True,quasiStatic=True)
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
if len(configs) > 20 :
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
fullBody.setJointBounds('leg_left_1_joint',joint1L_bounds_prev)
fullBody.setJointBounds('leg_left_6_joint',joint6L_bounds_prev)
fullBody.setJointBounds('leg_left_2_joint',joint2L_bounds_prev)
fullBody.setJointBounds('leg_right_1_joint',joint1R_bounds_prev)
fullBody.setJointBounds('leg_right_6_joint',joint6R_bounds_prev)
fullBody.setJointBounds('leg_right_2_joint',joint2R_bounds_prev)

