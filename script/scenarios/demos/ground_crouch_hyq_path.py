from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
	rootJointType = 'freeflyer'
	packageName = 'hpp-rbprm-corba'
	meshPackageName = 'hpp-rbprm-corba'
	# URDF file describing the trunk of the robot HyQ
	urdfName = 'hyq_trunk_large'
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
urdfName = 'hyq_trunk'
urdfNameRom = ['hyq_lhleg_rom','hyq_lfleg_rom','hyq_rfleg_rom','hyq_rhleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-6,5, -4, 4, 0.6, 2])
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support',])
rbprmBuilder.boundSO3([-0.1,0.1,-1,1,-1,1])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-5, 0, 0.63]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
q_goal = q_init [::]
q_goal [0:3] = [-2.5, 0, 0.63]; #pass the hole
#~ q_goal [0:3] = [5, 0, 0.63]; r (q_goal) #pass the bridge

ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, "groundcrouch", "planning", r)
#~ afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

# Choosing RBPRM shooter and path validation methods.
# Note that the standard RRT algorithm is used.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
t = ps.solve ()


if isinstance(t, list):
	t = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]

from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

#~ pp (0)
#~ pp (1)
r (q_init)

q_far = q_init [::]
q_far [0:3] = [-2, -3, 0.63]; 
r(q_far)

for i in range(1,10):
	rbprmBuilder.client.basic.problem.optimizePath(i)
	
        
cl = Client()
cl.problem.selectProblem("rbprm_path")
rbprmBuilder2 = Robot ("toto")
ps2 = ProblemSolver( rbprmBuilder2 )
cl.problem.selectProblem("default")
cl.problem.movePathToProblem(3,"rbprm_path",rbprmBuilder.getAllJointNames())
r2 = Viewer (ps2)
r2(q_far)
