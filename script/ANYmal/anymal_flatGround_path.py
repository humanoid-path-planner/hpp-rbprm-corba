from hpp.corbaserver.rbprm.anymal_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time


vMax = 0.3  # linear velocity bound for the root
aMax = 1.0  # linear acceleration bound for the root
extraDof = 6
mu = 0.5  # coefficient of friction
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot()
# Define bounds for the root : bounding box of the scenario
root_bounds = [-3, 3, -3, 3, 0.4, 0.5]
rbprmBuilder.setJointBounds("root_joint", root_bounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(rbprmBuilder.urdfNameRom)
for rom in rbprmBuilder.urdfNameRom:
    rbprmBuilder.setAffordanceFilter(rom, ["Support"])

# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-4.0, 4.0, -0.1, 0.1, -0.1, 0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds(
    [-vMax, vMax, -vMax, vMax, 0, 0, -aMax, aMax, -aMax, aMax, 0, 0]
)
indexECS = (
    rbprmBuilder.getConfigSize()
    - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()
)

# Creating an instance of HPP problem solver
ps = ProblemSolver(rbprmBuilder)
# define parameters used by various methods :
ps.setParameter("Kinodynamic/velocityBound", vMax)
ps.setParameter("Kinodynamic/accelerationBound", aMax)
# force the orientation of the trunk to match the direction of the motion
ps.setParameter("Kinodynamic/forceAllOrientation", True)
ps.setParameter("DynamicPlanner/sizeFootX", 0.01)
ps.setParameter("DynamicPlanner/sizeFootY", 0.01)
ps.setParameter("DynamicPlanner/friction", mu)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF", False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops", 100)

# initialize the viewer :
from hpp.gepetto import ViewerFactory

vf = ViewerFactory(ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool

afftool = AffordanceTool()
afftool.setAffordanceConfig("Support", [0.5, 0.03, 0.00005])
afftool.loadObstacleModel("hpp_environments", "multicontact/ground", "planning", vf)
v = vf.createViewer(displayArrows=True)
# afftool.visualiseAffordances('Support', v, v.color.lightBrown)
# v.addLandmark(v.sceneName,1)

# Setting initial configuration
q_init = rbprmBuilder.getCurrentConfig()
q_init[0:3] = [0, 0, 0.465]
q_init[-6:-3] = [0, 0, 0]
v(q_init)
ps.setInitialConfig(q_init)
# set goal config
rbprmBuilder.setCurrentConfig(q_init)
q_goal = q_init[::]
q_goal[0:3] = [1.0, 0, 0.465]
q_goal[-6:-3] = [0, 0, 0]
v(q_goal)


ps.addGoalConfig(q_goal)

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation", 0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :
t = ps.solve()
print("Guide planning time : ", t)


# display solution :
from hpp.gepetto import PathPlayer

pp = PathPlayer(v)
pp.dt = 0.1
# pp.displayVelocityPath(1)
# v.client.gui.setVisibility("path_1_root","ALWAYS_ON_TOP")
pp.dt = 0.01
# pp(1)

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)
