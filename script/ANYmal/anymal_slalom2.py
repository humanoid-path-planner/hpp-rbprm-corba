from hpp.corbaserver.rbprm.anymal import Robot
from hpp.gepetto import Viewer
from tools.display_tools import *
import time

print("Plan guide trajectory ...")
import anymal_slalom2_path as tp

print("Done.")
import time


pId = tp.ps.numberPaths() - 1
fullBody = Robot()
# Set the bounds for the root, take slightly larger bounding box than during root planning
root_bounds = tp.root_bounds[::]
root_bounds[0] -= 0.5
root_bounds[1] += 0.5
root_bounds[2] -= 0.5
root_bounds[3] += 0.5
root_bounds[-1] = 0.6
root_bounds[-2] = 0.3
fullBody.setJointBounds("root_joint", root_bounds)
fullBody.setVeryConstrainedJointsBounds()
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds(
    [
        -tp.vMax,
        tp.vMax,
        -tp.vMax,
        tp.vMax,
        0,
        0,
        -tp.aMax,
        tp.aMax,
        -tp.aMax,
        tp.aMax,
        0,
        0,
    ]
)
ps = tp.ProblemSolver(fullBody)
ps.setParameter("Kinodynamic/velocityBound", tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound", tp.aMax)
# load the viewer
v = tp.Viewer(ps, viewerClient=tp.v.client, displayCoM=True)

# load a reference configuration
q_ref = fullBody.referenceConfig[::] + [0] * 6
# q_init = fullBody.referenceConfig_asymetric[::]+[0]*6
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig(q_init)
fullBody.setPostureWeights(fullBody.postureWeights[::] + [0] * 6)
# fullBody.usePosturalTaskContactCreation(True)

print("Generate limb DB ...")
tStart = time.time()
# dict_heuristic = {fullBody.rLegId:"fixedStep04", fullBody.lLegId:"fixedStep04", fullBody.rArmId:"static", fullBody.lArmId:"static"}
fullBody.loadAllLimbs(
    "static", "ReferenceConfiguration", nbSamples=100000, disableEffectorCollision=False
)
# fullBody.loadAllLimbs(dict_heuristic,"ReferenceConfiguration",nbSamples=100000,disableEffectorCollision=False)
tGenerate = time.time() - tStart
print("Done.")
print("Databases generated in : " + str(tGenerate) + " s")
fullBody.setReferenceConfig(q_ref)

# define initial and final configurations :
configSize = (
    fullBody.getConfigSize() - fullBody.client.robot.getDimensionExtraConfigSpace()
)

q_init[0:7] = tp.ps.configAtParam(pId, 0.0)[
    0:7
]  # use this to get the correct orientation
q_goal = q_ref[::]
q_goal[0:7] = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[0:7]
dir_init = [0, 0, 0]
acc_init = tp.ps.configAtParam(pId, 0.0)[tp.indexECS + 3 : tp.indexECS + 6]
dir_goal = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[
    tp.indexECS : tp.indexECS + 3
]
acc_goal = [0, 0, 0]

robTreshold = 5
# copy extraconfig for start and init configurations
q_init[configSize : configSize + 3] = dir_init[::]
q_init[configSize + 3 : configSize + 6] = acc_init[::]
q_goal[configSize : configSize + 3] = dir_goal[::]
q_goal[configSize + 3 : configSize + 6] = [0, 0, 0]

# q_init[2] = fullBody.referenceConfig_asymetric[2]
q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]

normals = [[0.0, 0.0, 1.0] for _ in range(4)]
fullBody.setStaticStability(True)
fullBody.setCurrentConfig(q_init)
v(q_init)
fullBody.setStartState(q_init, Robot.limbs_names, normals)
fullBody.setEndState(q_goal, Robot.limbs_names, normals)

fullBody.setCurrentConfig(q_goal)
v(q_goal)


v.addLandmark("anymal/base", 0.3)


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(
    0.001,
    pathId=pId,
    robustnessTreshold=robTreshold,
    filterStates=True,
    testReachability=True,
    quasiStatic=True,
)
tInterpolateConfigs = time.time() - tStart
print("Done.")
print("Contact plan generated in : " + str(tInterpolateConfigs) + " s")
print("number of configs :", len(configs))
# raw_input("Press Enter to display the contact sequence ...")
# displayContactSequence(v,configs)


fullBody.resetJointsBounds()
