from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
from hpp.corbaserver import Client
from hpp.corbaserver.robot import Robot as Parent
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
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
rbprmBuilder.setJointBounds ("base_joint_xyz", [0,3, -2, 0, 0.3, 1])
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])
rbprmBuilder.setAffordanceFilter('hrp2_rarm_rom', ['Lean'])
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
ps.client.problem.setParameter("aMax",omniORB.any.to_any(aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(vMax))
ps.client.problem.setParameter("sizeFootX",omniORB.any.to_any(0.24))
ps.client.problem.setParameter("sizeFootY",omniORB.any.to_any(0.14))
r = Viewer (ps)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.setAffordanceConfig('Lean', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "roomWall", "planning", r)
r.addLandmark(r.sceneName,1)
afftool.visualiseAffordances('Support', r,r.color.brown)
afftool.visualiseAffordances('Lean', r, r.color.blue)


q_init = rbprmBuilder.getCurrentConfig ();
#q_init[0:3] = [2, -1, 0.58];
#q_init[0:3] = [2, -1, 0.58];
q_init[0:3] = [1.85, -1, 0.58];
q_init[3:7] = [0.7071,0,0,0.7071]
#q_init[indexECS:indexECS+3] = [2,0,0]
q_init[indexECS:indexECS+3] = [1,0,0]
rbprmBuilder.setCurrentConfig (q_init); r (q_init)


q_goal = q_init [::]
q_goal [0:3] = [1.2, -1, 0.58]; 
q_goal[indexECS:indexECS+3] = [-1,0,0]


#r(q_goal)


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")

#t = ps.solve ()


ps.client.problem.prepareSolveStepByStep()


ps.client.problem.finishSolveStepByStep()





# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=1./30.
#r.client.gui.removeFromGroup("rm0",r.sceneName)
pp.displayVelocityPath(0)
pp.speed=0.5
#pp(0)





q_far = q_init[::]
q_far[2] = -3
r(q_far)

#~ pp(0)




