from hpp.corbaserver.rbprm.anymal import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.tools.display_tools import *
import time

print("Plan guide trajectory ...")
import anymal_plinth_path as tp

print("Done.")
import time


pId = tp.ps.numberPaths() - 1
fullBody = Robot()
# Set the bounds for the root, take slightly larger bounding box than during root planning
root_bounds = tp.root_bounds[::]
root_bounds[0] -= 0.2
root_bounds[1] += 0.2
root_bounds[2] -= 0.2
root_bounds[3] += 0.2
root_bounds[-1] = 0.12
root_bounds[-2] = 0.3
fullBody.setJointBounds("root_joint", root_bounds)
fullBody.setConstrainedJointsBounds()
fullBody.setJointBounds("LF_KFE", [-1.4, 0.0])
fullBody.setJointBounds("RF_KFE", [-1.4, 0.0])
fullBody.setJointBounds("LH_KFE", [0.0, 1.4])
fullBody.setJointBounds("RH_KFE", [0.0, 1.4])
# add the 6 extraDof for velocity and acceleration (see *_path.py script)
fullBody.client.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.robot.setExtraConfigSpaceBounds(
    [
        -tp.vMax,
        tp.vMax,
        -tp.vMax,
        tp.vMax,
        -tp.vMax,
        tp.vMax,
        -tp.aMax,
        tp.aMax,
        -tp.aMax,
        tp.aMax,
        -tp.aMaxZ,
        tp.aMaxZ,
    ]
)
ps = tp.ProblemSolver(fullBody)
ps.setParameter("Kinodynamic/velocityBound", tp.vMax)
ps.setParameter("Kinodynamic/accelerationBound", tp.aMax)
# load the viewer
v = tp.Viewer(ps, viewerClient=tp.v.client, displayCoM=True)

# load a reference configuration
q_ref = fullBody.referenceConfig[::] + [0, 0, 0, 0, 0, 0]
q_init = q_ref[::]
fullBody.setReferenceConfig(q_ref)
fullBody.setCurrentConfig(q_init)
fullBody.setPostureWeights(fullBody.postureWeights[::] + [0] * 6)
fullBody.usePosturalTaskContactCreation(True)

dict_heuristic = {
    fullBody.rLegId: "static",
    fullBody.lLegId: "static",
    fullBody.rArmId: "fixedStep04",
    fullBody.lArmId: "fixedStep04",
}
print("Generate limb DB ...")
tStart = time.time()
fullBody.loadAllLimbs(dict_heuristic, "ReferenceConfiguration", nbSamples=50000)
tGenerate = time.time() - tStart
print("Done.")
print("Databases generated in : " + str(tGenerate) + " s")
fullBody.setReferenceConfig(q_ref)

# define initial and final configurations :
configSize = (
    fullBody.getConfigSize() - fullBody.client.robot.getDimensionExtraConfigSpace()
)

q_init[0:7] = tp.ps.configAtParam(pId, 0)[
    0:7
]  # use this to get the correct orientation
q_goal = q_init[::]
q_goal[0:7] = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId, 0)[tp.indexECS : tp.indexECS + 3]
acc_init = tp.ps.configAtParam(pId, 0)[tp.indexECS + 3 : tp.indexECS + 6]
dir_goal = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[
    tp.indexECS : tp.indexECS + 3
]
acc_goal = [0, 0, 0]

robTreshold = 2
# copy extraconfig for start and init configurations
q_init[configSize : configSize + 3] = dir_init[::]
q_init[configSize + 3 : configSize + 6] = acc_init[::]
q_goal[configSize : configSize + 3] = dir_goal[::]
q_goal[configSize + 3 : configSize + 6] = [0, 0, 0]

q_init[2] = q_ref[2]
q_goal[2] = q_ref[2]

fullBody.setStaticStability(True)
fullBody.setCurrentConfig(q_init)
v(q_init)

fullBody.setCurrentConfig(q_goal)
v(q_goal)

v.addLandmark("anymal_reachability/base", 0.3)
v(q_init)

# specify the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init, fullBody.limbs_names)
fullBody.setEndState(q_goal, fullBody.limbs_names)


print("Generate contact plan ...")
tStart = time.time()
configs = fullBody.interpolate(
    0.002,
    pathId=pId,
    robustnessTreshold=robTreshold,
    filterStates=True,
    testReachability=True,
    quasiStatic=True,
)
tInterpolateConfigs = time.time() - tStart
print("Done. ")
print("Contact sequence computed in " + str(tInterpolateConfigs) + " s.")
print("number of configs :", len(configs))
# raw_input("Press Enter to display the contact sequence ...")
# displayContactSequence(v,configs)

fullBody.resetJointsBounds()
