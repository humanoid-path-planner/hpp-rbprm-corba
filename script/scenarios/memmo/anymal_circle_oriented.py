from hpp.corbaserver.rbprm.anymal import Robot
from hpp.gepetto import Viewer
from tools.display_tools import *
import time
print "Plan guide trajectory ..."
import anymal_circle_oriented_path as tp
print "Done."
import time
statusFilename = tp.statusFilename
pId = 0
f = open(statusFilename,"a")
if tp.ps.numberPaths() > 0 :
  print "Path planning OK."
  f.write("Planning_success: True"+"\n")
  f.close()
else :
  print "Error during path planning"
  f.write("Planning_success: False"+"\n")
  f.close()
  import sys
  sys.exit(1)


fullBody = Robot ()
root_bounds =  tp.root_bounds
root_bounds[-1] = 0.6
root_bounds[-2] = 0.3
# Set the bounds for the root
fullBody.setJointBounds ("root_joint",  root_bounds)
## reduce bounds on joints along x, to put conservative condition on the contact generation for sideway steps
fullBody.setVeryConstrainedJointsBounds()
fullBody.setJointBounds('LF_HAA',[-0.2,0.2])
fullBody.setJointBounds('RF_HAA',[-0.2,0.2])
fullBody.setJointBounds('LH_HAA',[-0.2,0.2])
fullBody.setJointBounds('RH_HAA',[-0.2,0.2])

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
    print "No viewer started !"
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
fullBody.usePosturalTaskContactCreation(True)

"""
if abs(tp.q_goal[1]) <= abs(tp.q_goal[0]) : 
  heuristicR = "fixedStep08"
  heuristicL = "fixedStep08"
  print "Use weight for straight walk"
  fullBody.usePosturalTaskContactCreation(True)
else :
  print "Use weight for straff walk"
  if tp.q_goal[1] < 0 :
    print "start with right leg"
    heuristicL = "static"
    heuristicR = "fixedStep06"
  else:
    print "start with left leg"
    heuristicR = "static"
    heuristicL = "fixedStep06"
"""

fullBody.setCurrentConfig (q_init)

print "Generate limb DB ..."
tStart = time.time()
# generate databases : 
"""
nbSamples = 100000
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, heuristicR, 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, heuristicL, 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)
"""
fullBody.loadAllLimbs("fixedStep04")

tGenerate =  time.time() - tStart
print "Done."
print "Databases generated in : "+str(tGenerate)+" s"

#define initial and final configurations : 
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0)[0:7] # use this to get the correct orientation
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
vel_init = tp.ps.configAtParam(pId,0)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0)[tp.indexECS+3:tp.indexECS+6]
vel_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = vel_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = vel_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]


fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
v(q_init)

fullBody.setCurrentConfig (q_goal)
v(q_goal)

v.addLandmark('anymal/base_0',0.3)
v(q_init)
#fullBody.setReferenceConfig(fullBody.referenceConfig_legsApart[::]+[0]*6)


# specify the full body configurations as start and goal state of the problem

normals = [[0.,0.,1.] for _ in range(4)]
if q_goal[1] < 0: # goal on the right side of the circle, start motion with right leg first
  fullBody.setStartState(q_init,[fullBody.rArmId,fullBody.rLegId,fullBody.lArmId,fullBody.lLegId],normals)
  fullBody.setEndState(q_goal,[fullBody.rArmId,fullBody.rLegId,fullBody.lArmId,fullBody.lLegId],normals)
else :
  fullBody.setStartState(q_init,[fullBody.lArmId,fullBody.lLegId,fullBody.rArmId,fullBody.rLegId],normals)
  fullBody.setEndState(q_goal,[fullBody.lArmId,fullBody.lLegId,fullBody.rArmId,fullBody.rLegId],normals)

print "Generate contact plan ..."
tStart = time.time()
configs = fullBody.interpolate(0.001,pathId=pId,robustnessTreshold = 1, filterStates = True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print "Done."
print "Contact plan generated in : "+str(tInterpolateConfigs)+" s"
print "number of configs :", len(configs)
#raw_input("Press Enter to display the contact sequence ...")
#displayContactSequence(v,configs)


if len(configs) < 2 :
  cg_success = False
  print "Error during contact generation."
else:
  cg_success = True
  print "Contact generation Done."
if abs(configs[-1][0] - tp.q_goal[0]) < 0.01 and abs(configs[-1][1]- tp.q_goal[1]) < 0.01  and (len(fullBody.getContactsVariations(len(configs)-2,len(configs)-1))==1):
  print "Contact generation successful."
  cg_reach_goal = True
else:
  print "Contact generation failed to reach the goal."
  cg_reach_goal = False
if len(configs) > 50 :
  cg_too_many_states = True
  cg_success = False
  print "Discarded contact sequence because it was too long."
else:
  cg_too_many_states = False

f = open(statusFilename,"a")
f.write("cg_success: "+str(cg_success)+"\n")
f.write("cg_reach_goal: "+str(cg_reach_goal)+"\n")
f.write("cg_too_many_states: "+str(cg_too_many_states)+"\n")
f.close()

if (not cg_success) or cg_too_many_states or (not cg_reach_goal):
  import sys
  sys.exit(1)

# put back original bounds for wholebody methods
fullBody.resetJointsBounds()
