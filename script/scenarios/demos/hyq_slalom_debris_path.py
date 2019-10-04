from hpp.corbaserver.rbprm.hyq_abstract import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver import ProblemSolver
import time



vMax = 0.2# linear velocity bound for the root
aMax = 0.1# linear acceleration bound for the root
extraDof = 6
mu=0.5# coefficient of friction
# Creating an instance of the helper class, and loading the robot
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Robot ()
# Define bounds for the root : bounding box of the scenario
root_bounds = [-1.7,2.5, -0.2, 2, 0.63, 0.63]
rbprmBuilder.setJointBounds ("root_joint",root_bounds)

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact with the corresponding afforcances type
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-4,4,-0.1,0.1,-0.1,0.1])
# Add 6 extraDOF to the problem, used to store the linear velocity and acceleration of the root
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
# We set the bounds of this extraDof with velocity and acceleration bounds (expect on z axis)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver 
ps = ProblemSolver( rbprmBuilder )
# define parameters used by various methods : 
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("DynamicPlanner/sizeFootX",0.01)
ps.setParameter("DynamicPlanner/sizeFootY",0.01)
ps.setParameter("DynamicPlanner/friction",0.5)
ps.setParameter("Kinodynamic/forceYawOrientation",True)
# sample only configuration with null velocity and acceleration :
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)
ps.setParameter("PathOptimization/RandomShortcut/NumberOfLoops",50)

# initialize the viewer :
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

# load the module to analyse the environnement and compute the possible contact surfaces
from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp_environments", "multicontact/slalom_debris", "planning", vf,reduceSizes=[0.05,0,0])
v = vf.createViewer(displayArrows = True)
#afftool.visualiseAffordances('Support', v, v.color.lightBrown)

# Setting initial configuration
q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-1.5, 0, 0.63]
q_init[3:7] = [0,0,0,1]
q_init[-6] = 0.05
v (q_init)
ps.setInitialConfig (q_init)
# set goal config
rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]
q_goal[0] = 2.2
v(q_goal)
ps.addGoalConfig (q_goal)

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")

# Solve the planning problem :
t = ps.solve ()
print "Guide planning time : ",t

ps.optimizePath(1)
pid = ps.numberPaths()-1
# display solution : 
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
pp.dt=0.1
#pp.displayVelocityPath(pid)
#v.client.gui.setVisibility("path_2_root","ALWAYS_ON_TOP")
pp.dt=0.01
#pp(0)

# move the robot out of the view before computing the contacts
q_far = q_init[::]
q_far[2] = -2
v(q_far)

