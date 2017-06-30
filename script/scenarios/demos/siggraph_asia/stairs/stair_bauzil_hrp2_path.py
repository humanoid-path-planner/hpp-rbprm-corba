from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent

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
rbprmBuilder.setJointBounds ("base_joint_xyz", [0,2.5, -1, 1, 0, 2.2])
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])
#~ rbprmBuilder.setAffordanceFilter('hrp2_larm_rom', ['Support','Lean'])
rbprmBuilder.setAffordanceFilter('hrp2_rarm_rom', ['Support','Lean'])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support'])
#~ rbprmBuilder.setNormalFilter('hrp2_rarm_rom', [0,0,1], 0.5)
#~ rbprmBuilder.setNormalFilter('hrp2_lleg_rom', [0,0,1], 0.9)
#~ rbprmBuilder.setNormalFilter('hrp2_rleg_rom', [0,0,1], 0.9)
#~ rbprmBuilder.setNormalFilter('hyq_rhleg_rom', [0,0,1], 0.9)
rbprmBuilder.boundSO3([-0.,0,-1,1,-1,1])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [0, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
q_init [0:3] = [0.1, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [0, -0.63, 0.6]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]

q_goal = q_init [::]
q_goal [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
#~ q_goal [0:3] = [1.49, -0.65, 1.25]; r (q_goal)
q_goal [0:3] = [2.1, -0.82, 1.0]; r (q_goal)

#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "stairs_lower", "planning", r)
#~ afftool.analyseAll()
#~ afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
#~ afftool.visualiseAffordances('Lean', r, [0, 0, 0.9])

#~ ps.client.problem.selectConFigurationShooter("RbprmShooter")
#~ ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
#~ ps.solve ()
t = ps.solve ()

print t;
if isinstance(t, list):
	t = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)

#~ pp (1)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.01, "stair_bauzil_hrp2_path.txt")

cl = Client()
cl.problem.selectProblem("rbprm_path")
rbprmBuilder2 = Robot ("toto")
ps2 = ProblemSolver( rbprmBuilder2 )
cl.problem.selectProblem("default")
cl.problem.movePathToProblem(1,"rbprm_path",rbprmBuilder.getAllJointNames())
r2 = Viewer (ps2, viewerClient=r.client)
r.client.gui.setVisibility("toto", "OFF")
#~ r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
#~ r2(q_far)
