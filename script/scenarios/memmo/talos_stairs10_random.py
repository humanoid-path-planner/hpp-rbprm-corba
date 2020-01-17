from talos_rbprm.talos import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import time
print("Plan guide trajectory ...")
import talos_stairs10_random_path as tp
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



pId = tp.pId
fullBody = Robot ()
# Set the bounds for the root, take slightly larger bounding box than during root planning
root_bounds = tp.root_bounds[::]
root_bounds[0] -= 0.2
root_bounds[1] += 0.2
root_bounds[2] -= 0.2
root_bounds[3] += 0.2
root_bounds[-1] = 1.9
root_bounds[-2] = 0.8
fullBody.setJointBounds ("root_joint",  root_bounds)
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,-10.,10.,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,-10.,10.])
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
q_ref = fullBody.referenceConfig[::]+[0,0,0,0,0,0]
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig (q_init)
fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)
fullBody.usePosturalTaskContactCreation(True)
print("Generate limb DB ...")
tStart = time.time()
# generate databases :
nbSamples = 50000
heuristic="fixedStep06"
fullBody.addLimb(fullBody.rLegId,fullBody.rleg,fullBody.rfoot,fullBody.rLegOffset,fullBody.rLegNormal, fullBody.rLegx, fullBody.rLegy, nbSamples, heuristic, 0.01,kinematicConstraintsPath=fullBody.rLegKinematicConstraints,kinematicConstraintsMin = 0.85)
fullBody.addLimb(fullBody.lLegId,fullBody.lleg,fullBody.lfoot,fullBody.lLegOffset,fullBody.rLegNormal, fullBody.lLegx, fullBody.lLegy, nbSamples, heuristic, 0.01,kinematicConstraintsPath=fullBody.lLegKinematicConstraints,kinematicConstraintsMin = 0.85)
#fullBody.runLimbSampleAnalysis(fullBody.rLegId, "ReferenceConfiguration", True)
#fullBody.runLimbSampleAnalysis(fullBody.lLegId, "ReferenceConfiguration", True)


tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")


#define initial and final configurations :
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()

q_init[0:7] = tp.ps.configAtParam(pId,0.)[0:7] # use this to get the correct orientation
q_goal = q_init[::]; q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId,0.)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0.)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 2
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]

q_init[2]+=0.0432773
q_goal[2]+=0.0432773

fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
v(q_init)

fullBody.setCurrentConfig (q_goal)
v(q_goal)

v.addLandmark('talos/base_link',0.3)
v(q_init)

# specify the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[fullBody.lLegId,fullBody.rLegId])
fullBody.setEndState(q_goal,[fullBody.lLegId,fullBody.rLegId])


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(0.005,pathId=pId,robustnessTreshold = 2, filterStates = True,testReachability=True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print("Done.")
print("number of configs :", len(configs))

if len(configs) < 8 :
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
if len(configs) > 30 :
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

