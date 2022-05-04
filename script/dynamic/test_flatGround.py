from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State, StateHelper
from hpp.corbaserver.rbprm.tools.display_tools import *
import flatGround_hrp2_pathKino as tp
import time

tPlanning = tp.tPlanning


packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""
pId = tp.ps.numberPaths() - 1
fullBody = FullBody()

fullBody.loadFullBodyModel(
    urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix
)
fullBody.setJointBounds("base_joint_xyz", [-5, 5, -1.5, 1.5, 0.5, 0.8])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds(
    [-0, 0, -0, 0, -0, 0, 0, 0, 0, 0, 0, 0]
)
ps = tp.ProblemSolver(fullBody)
ps.client.problem.setParameter("aMax", tp.aMax)
ps.client.problem.setParameter("vMax", tp.vMax)
r = tp.Viewer(ps, viewerClient=tp.r.client, displayArrows=True, displayCoM=True)


q_init = [
    0.1,
    -0.82,
    0.648702,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.261799388,
    0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,
    0.261799388,
    -0.174532925,
    0.0,
    -0.523598776,
    0.0,
    0.0,
    0.17,
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,
    0.0,
    0.0,
    -0.453785606,
    0.872664626,
    -0.41887902,
    0.0,
    0,
    0,
    0,
    0,
    0,
    0,
]
r(q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig(q_init)
qfar = q_ref[::]
qfar[2] = -5

# ~ AFTER loading obstacles
rLegId = "hrp2_rleg_rom"
lLegId = "hrp2_lleg_rom"
tStart = time.time()
fullBody.setReferenceConfig(q_ref)

rLeg = "RLEG_JOINT0"
rLegOffset = [0, 0, -0.105]
rLegLimbOffset = [0, 0, -0.035]  # 0.035
rLegNormal = [0, 0, 1]
rLegx = 0.09
rLegy = 0.05
# fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(
    rLegId,
    rLeg,
    "",
    rLegOffset,
    rLegNormal,
    rLegx,
    rLegy,
    100000,
    "fixedStep1",
    0.01,
    "_6_DOF",
    limbOffset=rLegLimbOffset,
)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
# fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = "LLEG_JOINT0"
lLegOffset = [0, 0, -0.105]
lLegLimbOffset = [0, 0, 0.035]
lLegNormal = [0, 0, 1]
lLegx = 0.09
lLegy = 0.05
# fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(
    lLegId,
    lLeg,
    "",
    lLegOffset,
    rLegNormal,
    lLegx,
    lLegy,
    100000,
    "fixedStep1",
    0.01,
    "_6_DOF",
    limbOffset=lLegLimbOffset,
)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
# fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")
fullBody.setReferenceConfig(q_ref)
## Add arms (not used for contact) :


rarmId = "hrp2_rarm_rom"
rarm = "RARM_JOINT0"
rHand = "RARM_JOINT5"
fullBody.addNonContactingLimb(rarmId, rarm, rHand, 10000)
fullBody.runLimbSampleAnalysis(rarmId, "ReferenceConfiguration", True)
larmId = "hrp2_larm_rom"
larm = "LARM_JOINT0"
lHand = "LARM_JOINT5"
fullBody.addNonContactingLimb(larmId, larm, lHand, 10000)
fullBody.runLimbSampleAnalysis(larmId, "ReferenceConfiguration", True)


tGenerate = time.time() - tStart
print("generate databases in : " + str(tGenerate) + " s")


q_0 = fullBody.getCurrentConfig()
# ~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)


configSize = (
    fullBody.getConfigSize()
    - fullBody.client.basic.robot.getDimensionExtraConfigSpace()
)

q_init = fullBody.getCurrentConfig()
q_init[0:7] = tp.ps.configAtParam(pId, 0.01)[
    0:7
]  # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig()
q_goal[0:7] = tp.ps.configAtParam(pId, tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId, 0.01)[tp.indexECS : tp.indexECS + 3]
acc_init = tp.ps.configAtParam(pId, 0.01)[tp.indexECS + 3 : tp.indexECS + 6]
dir_goal = tp.ps.configAtParam(pId, tp.ps.pathLength(pId) - 0.01)[
    tp.indexECS : tp.indexECS + 3
]
acc_goal = [0, 0, 0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize : configSize + 3] = dir_init[::]
q_init[configSize + 3 : configSize + 6] = acc_init[::]
q_goal[configSize : configSize + 3] = dir_goal[::]
q_goal[configSize + 3 : configSize + 6] = [0, 0, 0]


# FIXME : test
q_init[2] = q_ref[2] + 0.01
q_goal[2] = q_ref[2] + 0.01

fullBody.setStaticStability(True)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig(q_init)
r(q_init)
# q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)


fullBody.isDynamicallyReachableFromState(s.sId, s2.sId, timings=[0.4, 0.6, 0.4])
