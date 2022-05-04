# Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = "freeflyer"
packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
urdfName = "hyq_trunk_large"
urdfNameRom = ["hyq_lhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_rhleg_rom"]
urdfSuffix = ""
srdfSuffix = ""

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
rbprmBuilder.setJointBounds("base_joint_xyz", [-2, 5, -1, 1, 0.3, 4])

rbprmBuilder.setFilter(
    ["hyq_lhleg_rom", "hyq_lfleg_rom", "hyq_rfleg_rom", "hyq_rhleg_rom"]
)
rbprmBuilder.setAffordanceFilter("hyq_rhleg_rom", ["Support", "Lean"])
rbprmBuilder.setAffordanceFilter(
    "hyq_rfleg_rom",
    [
        "Support",
    ],
)
rbprmBuilder.setAffordanceFilter("hyq_lhleg_rom", ["Support", "Lean"])
rbprmBuilder.setAffordanceFilter(
    "hyq_lfleg_rom",
    [
        "Support",
    ],
)
# We also bound the rotations of the torso.
rbprmBuilder.boundSO3([-0.4, 0.4, -3, 3, -3, 3])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver(rbprmBuilder)
r = Viewer(ps)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig()
q_init[0:3] = [-2, 0, 0.63]
rbprmBuilder.setCurrentConfig(q_init)
r(q_init)
q_goal = q_init[::]
q_goal[0:3] = [3, 0, 0.63]
r(q_goal)

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

from hpp.corbaserver.affordance.affordance import AffordanceTool

afftool = AffordanceTool()
afftool.loadObstacleModel(packageName, "darpa", "planning", r)
afftool.visualiseAffordances("Support", r, [0.25, 0.5, 0.5])

# Choosing RBPRM shooter and path validation methods.
# Note that the standard RRT algorithm is used.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation", 0.05)

# Solve the problem
t = ps.solve()

# Playing the computed path
from hpp.gepetto import PathPlayer

pp = PathPlayer(rbprmBuilder.client.basic, r)
pp(0)
