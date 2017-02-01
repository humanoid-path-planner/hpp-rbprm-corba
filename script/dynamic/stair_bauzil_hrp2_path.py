from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver


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
#rbprmBuilder.setJointBounds ("base_joint_xyz", [0,1.5, -1, 0, 0.6, 1.3])
rbprmBuilder.setJointBounds ("base_joint_xyz", [0,2, -1, 1, 0, 2.2])
rbprmBuilder.setFilter(['hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom'])
rbprmBuilder.setAffordanceFilter('hrp2_rarm_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
rbprmBuilder.boundSO3([-0.,0,-1,1,-1,1])
vMax = 1;
aMax = 10;
extraDof = 6
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,0,0,0,0,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()

#~ from hpp.corbaserver.rbprm. import ProblemSolver


ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",aMax)
ps.client.problem.setParameter("vMax",vMax)
ps.client.problem.setParameter("sizeFootX",0.24)
ps.client.problem.setParameter("sizeFootY",0.14)
r = Viewer (ps)




q_init = rbprmBuilder.getCurrentConfig ();

q_init [0:3] = [0.15, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
q_goal [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_goal [0:3] = [1.49, -0.65, 1.15]; r (q_goal)
#~ q_goal [0:3] = [1.2, -0.65, 1.1]; r (q_goal)



from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "stair_bauzil", "planning", r)

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


ps.solve()



# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.005
#r.client.gui.removeFromGroup("rm0",r.sceneName)
pp.displayVelocityPath(0)
pp.speed=0.2
pp(0)





q_far = q_init[::]
q_far[2] = -3
r(q_far)

