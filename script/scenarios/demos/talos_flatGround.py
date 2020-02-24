from talos_rbprm.talos import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import time
print("Plan guide trajectory ...")
from . import talos_flatGround_path as tp
print("Done.")
import time

pId = tp.ps.numberPaths() -1
fullBody = Robot ()

# Set the bounds for the root
fullBody.setJointBounds ("root_joint",  [-5,5, -1.5, 1.5, 0.95, 1.05])
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds([-tp.vMax,tp.vMax,-tp.vMax,tp.vMax,0,0,-tp.aMax,tp.aMax,-tp.aMax,tp.aMax,0,0])
ps = tp.ProblemSolver( fullBody )
ps.setParameter("Kinodynamic/velocityBound",tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound",tp.aMax)
#load the viewer
v = tp.Viewer (ps,viewerClient=tp.v.client, displayCoM = True)

# load a reference configuration
q_ref = fullBody.referenceConfig[::]+[0]*6
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig (q_ref)
fullBody.setPostureWeights(fullBody.postureWeights[::]+[0]*6)
fullBody.usePosturalTaskContactCreation(True)

print("Generate limb DB ...")
tStart = time.time()
# generate databases :
fullBody.limbs_names = tp.used_limbs
fullBody.loadAllLimbs("fixedStep06")
tGenerate =  time.time() - tStart
print("Done.")
print("Databases generated in : "+str(tGenerate)+" s")

#define initial and final configurations :
q_init = q_ref[::]
q_init[0:7] = tp.ps.configAtParam(pId,0.001)[0:7] 
q_goal = q_init[::]
q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]

# force root height to be at the reference position: 
q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]

# copy extraconfig for start and init configurations
configSize = fullBody.getConfigSize() -fullBody.client.robot.getDimensionExtraConfigSpace()
q_init[configSize:configSize+3] = tp.ps.configAtParam(pId,0)[tp.indexECS:tp.indexECS+3]
q_init[configSize+3:configSize+6] = tp.ps.configAtParam(pId,0)[tp.indexECS:tp.indexECS+3]
q_goal[configSize:configSize+3] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[tp.indexECS:tp.indexECS+3]
q_goal[configSize+3:configSize+6] = [0,0,0]


fullBody.setStaticStability(True)
v.addLandmark('talos/base_link',0.3)
v(q_init)

# specify the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[fullBody.rLegId,fullBody.lLegId])
fullBody.setEndState(q_goal,[fullBody.rLegId,fullBody.lLegId])


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = 2, filterStates = True,quasiStatic=True)
tInterpolateConfigs = time.time() - tStart
print("Done.")
print("Contact plan generated in : "+str(tInterpolateConfigs)+" s")
print("number of configs :", len(configs))
#raw_input("Press Enter to display the contact sequence ...")
#displayContactSequence(v,configs)



