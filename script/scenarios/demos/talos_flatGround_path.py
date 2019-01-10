from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time




rootJointType = 'freeflyer'
packageName = 'talos-rbprm'
meshPackageName = 'talos-rbprm'
urdfName = 'talos_trunk'
urdfNameRom =  ['talos_larm_rom','talos_rarm_rom','talos_lleg_rom','talos_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""
vMax = 0.3
aMax = 0.1
extraDof = 6
mu=0.5
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("root_joint", [-5,5, -1.5, 1.5, 0.95, 1.05])


# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['talos_lleg_rom','talos_rleg_rom'])
rbprmBuilder.setAffordanceFilter('talos_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('talos_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-1.7,1.7,-0.1,0.1,-0.1,0.1])
rbprmBuilder.client.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,-aMax,aMax,-aMax,aMax,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver and the viewer

ps = ProblemSolver( rbprmBuilder )
ps.setParameter("Kinodynamic/velocityBound",vMax)
ps.setParameter("Kinodynamic/accelerationBound",aMax)
ps.setParameter("DynamicPlanner/sizeFootX",0.2)
ps.setParameter("DynamicPlanner/sizeFootY",0.12)
ps.setParameter("DynamicPlanner/friction",0.5)
ps.setParameter("ConfigurationShooter/sampleExtraDOF",False)


from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel ("hpp-rbprm-corba", "ground", "planning", vf)
v = vf.createViewer(displayArrows = True)
afftool.visualiseAffordances('Support', v, v.color.lightBrown)

# Setting initial configuration
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [0,0,0,1]
q_init [0:3] = [0, 0, 1.]
v (q_init)

# set goal config
rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]
q_goal[0] = 1.5
v(q_goal)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# Choosing RBPRM shooter and path validation methods.
ps.selectConfigurationShooter("RbprmShooter")
ps.addPathOptimizer ("RandomShortcutDynamic")
ps.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods :
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("Kinodynamic")
ps.selectPathPlanner("DynamicPlanner")


t = ps.solve ()
print "Guide planning time : ",t


# display solution : 
from hpp.gepetto import PathPlayer
pp = PathPlayer (v)
pp.dt=0.03
pp.displayVelocityPath(0)
v.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
pp(0)


q_far = q_init[::]
q_far[2] = -2
v(q_far)

