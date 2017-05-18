from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from planning.configs.stairs_config import *
import omniORB.any

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-rbprm-corba'
	meshPackageName = 'hpp-rbprm-corba'
	# URDF file describing the trunk of the robot HyQ
	urdfName = 'hrp2_trunk_flexible'
	urdfSuffix = ""
	srdfSuffix = ""
	def __init__ (self, robotName, load = True):
		Parent.__init__ (self, robotName, self.rootJointType, load)
		self.tf_root = "base_footprint"
		self.client.basic = Client ()
		self.load = load
		

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms =  ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [0,1.55, -0.9, -0.55, 0.50, 1.3])
#rbprmBuilder.setJointBounds ("base_joint_xyz", [0,2, -1, 1, 0, 2.2])
rbprmBuilder.setJointBounds('CHEST_JOINT0',[0,0])
rbprmBuilder.setJointBounds('CHEST_JOINT1',[0,0.45])
rbprmBuilder.setJointBounds('HEAD_JOINT0',[0,0])
rbprmBuilder.setJointBounds('HEAD_JOINT1',[0,0])

rbprmBuilder.setFilter(['hrp2_rarm_rom'])
rbprmBuilder.setAffordanceFilter('hrp2_rarm_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
rbprmBuilder.boundSO3([-0.,0,-1,1,-1,1])
vMax = 0.1;
aMax = 5.;
extraDof = 6
mu=omniORB.any.to_any(MU)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-vMax,vMax,-0.2,0.2,-vMax,vMax,0,0,0,0,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()

#~ from hpp.corbaserver.rbprm. import ProblemSolver


ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(vMax))
ps.client.problem.setParameter("sizeFootX",omniORB.any.to_any(0.24))
ps.client.problem.setParameter("sizeFootY",omniORB.any.to_any(0.14))
r = Viewer (ps)
r.addLandmark(r.sceneName,1)





from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "stair_bauzil", "planning", r)




q_init = rbprmBuilder.getCurrentConfig ();

q_init [0:3] = [0, -0.82, 0.55];
q_init[8] = 0
#q_init [0:3] = [0, -0.65, 0.58];
#q_init[8] = 0.43
rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
q_goal [3:7] =  [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_goal[8] = 0
q_goal [0:3] = [1.49, -0.82, 1.25]; r (q_goal)
#~ q_goal [0:3] = [1.2, -0.65, 1.1]; r (q_goal)





ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")


#ps.client.problem.prepareSolveStepByStep()

#ps.client.problem.finishSolveStepByStep()

#r.solveAndDisplay("rm",1,0.01)

ps.solve()

# was 5.5


# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
#r.client.gui.removeFromGroup("rm",r.sceneName)
pp.displayVelocityPath(0)
pp.speed=0.3
#pp(0)



q_far = q_init[::]
q_far[2] = -3
r(q_far)

