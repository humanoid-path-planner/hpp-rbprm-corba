## Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer
import time

rootJointType = "freeflyer"
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
# URDF file describing the trunk of the robot HyQ
urdfName = "hyq_trunk_large"
# URDF files describing the reachable workspace of each limb of HyQ
urdfNameRom = ["hyq_lhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_rhleg_rom"]
urdfSuffix = ""
srdfSuffix = ""
vMax = 2
aMax = 5
extraDof = 6
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder()
rbprmBuilder.loadModel(
    urdfName,
    urdfNameRom,
    rootJointType,
    meshPackageName,
    packageName,
    urdfSuffix,
    srdfSuffix,
)
rbprmBuilder.setJointBounds("base_joint_xyz", [-2.5, 2.5, -6, 6, 0.6, 0.65])
# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(
    ["hyq_rhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_lhleg_rom"]
)
rbprmBuilder.setAffordanceFilter("hyq_rhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter(
    "hyq_rfleg_rom",
    [
        "Support",
    ],
)
rbprmBuilder.setAffordanceFilter("hyq_lhleg_rom", ["Support"])
rbprmBuilder.setAffordanceFilter(
    "hyq_lfleg_rom",
    [
        "Support",
    ],
)
# We also bound the rotations of the torso. (z, y, x)
# rbprmBuilder.boundSO3([-3,3,-0.1,0.1,-0.1,0.1])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds(
    [-vMax, vMax, -vMax, vMax, 0, 0, 0, 0, 0, 0, 0, 0]
)

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver(rbprmBuilder)
ps.client.problem.setParameter("aMax", aMax)
ps.client.problem.setParameter("vMax", vMax)
r = Viewer(ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool

afftool = AffordanceTool()
afftool.setAffordanceConfig("Support", [0.5, 0.03, 0.00005])
afftool.loadObstacleModel(packageName, "detour", "planning", r)
# r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances("Support", r, [0.25, 0.5, 0.5])
r.addLandmark(r.sceneName, 1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig()
q_init[3:7] = [1, 0, 0, 0]
q_init[0:3] = [2, 2, 0.63]
r(q_init)


rbprmBuilder.setCurrentConfig(q_init)
q_goal = q_init[::]

q_goal[3:7] = [1, 0, 0, 0]
q_goal[0:3] = [1.5, -4.5, 0.63]
r(q_goal)
q_goal[7:10] = [1.8, 0, 0]
r(q_goal)


# Choosing a path optimizer
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation", 0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")

# solve the problem :
r(q_init)

# ps.client.problem.prepareSolveStepByStep()

q_far = q_init[::]
q_far[2] = -3
r(q_far)

# r.solveAndDisplay("rm",1,0.01)


t = ps.solve()
if isinstance(t, list):
    t = t[0] * 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open("log.txt", "a")
f.write("path computation " + str(t) + "\n")
f.close()


# Playing the computed path
from hpp.gepetto import PathPlayer

pp = PathPlayer(rbprmBuilder.client.basic, r)
pp.dt = 0.03
pp.displayVelocityPath(0)
r.client.gui.setVisibility("path_0_root", "ALWAYS_ON_TOP")
# display path
pp.speed = 0.5
# pp (0)

# display path with post-optimisation

"""
ps.client.problem.extractPath(0,0,11.18)
r.client.gui.removeFromGroup("path_0_root",r.sceneName)
pp.displayVelocityPath(1)
r.client.gui.setVisibility("path_1_root","ALWAYS_ON_TOP")
#pp (1)
"""


"""
q_far = q_init[::]
q_far[2] = -3
r(q_far)
"""


"""
for i in range(1,10):
    rbprmBuilder.client.basic.problem.optimizePath(i)
    r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName)
    pp.displayVelocityPath(i+1)
    #time.sleep(2)
"""

"""
i=0

ps.clearRoadmap()
ps.solve()
r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName)
i = i+1
pp.displayVelocityPath(i)

pp(i)


"""

"""
r.client.gui.addCurve("c1",qlist,r.color.red)
r.client.gui.setVisibility("c1","ALWAYS_ON_TOP")
r.client.gui.addToGroup("c1",r.sceneName)


r.client.gui.addCurve("c2",qlist2,r.color.blue)
r.client.gui.setVisibility("c2","ALWAYS_ON_TOP")
r.client.gui.addToGroup("c2",r.sceneName)


"""


"""
nodes = ["hyq_trunk_large/base_link","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("yaml/hyq_slalom_path.yaml",nodes)
r.client.gui.captureTransformOnRefresh(True)
pp(0)
r.client.gui.captureTransformOnRefresh(False)

r.client.gui.writeNodeFile('path_0_root','/local/dev_hpp/screenBlender/iros2017/meshs/detour_path_kino.obj')

"""
