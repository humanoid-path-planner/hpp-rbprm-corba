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
urdfNameRom =  ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""
vMax = 4;
aMax = 6;
extraDof = 6

# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
#rbprmBuilder.setJointBounds ("base_joint_xyz", [-1.25,2, -0.5, 5.5, 0.6, 1.8])
rbprmBuilder.setJointBounds ("base_joint_xyz", [-2,4, 0, 2, 0.2, 1.4])
rbprmBuilder.setJointBounds('CHEST_JOINT0',[0,0])
rbprmBuilder.setJointBounds('CHEST_JOINT1',[-0.35,0.1])
rbprmBuilder.setJointBounds('HEAD_JOINT0',[0,0])
rbprmBuilder.setJointBounds('HEAD_JOINT1',[0,0])

# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-0.1,0.1,-0.65,0.65,-0.2,0.2])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-4,4,-1,1,-2,2,0,0,0,0,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()

# Creating an instance of HPP problem solver and the viewer

ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",aMax)
ps.client.problem.setParameter("vMax",vMax)
ps.client.problem.setParameter("sizeFootX",0.24)
ps.client.problem.setParameter("sizeFootY",0.14)
r = Viewer (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "downSlope_easy", "planning", r)
#r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
r.addLandmark(r.sceneName,1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [1,0,0,0]
q_init[8] = -0.2
q_init [0:3] = [-1.6, 1, 1.25]; r (q_init)

#q_init[3:7] = [0.7071,0,0,0.7071]
#q_init [0:3] = [1, 1, 0.65]

rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]


q_goal[3:7] = [1,0,0,0]
q_goal[8] = 0
q_goal [0:3] = [3, 1, 0.55]; r (q_goal)

r (q_goal)
#~ q_goal [0:3] = [-1.5, 0, 0.63]; r (q_goal)

# Choosing a path optimizer
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")

#solve the problem :
r(q_init)


#r.solveAndDisplay("rm",1,0.01)

t = ps.solve ()
if isinstance(t, list):
	t = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()


"""
for i in range(0,9):
  t = ps.solve()
  if isinstance(t, list):
    ts = t[0]* 3600. + t[1] * 60. + t[2] + t[3]/1000.	
  f= open("/local/dev_hpp/logs/benchHrp2_slope_LP.txt","a")
  f.write("t = "+str(ts) + "\n")
  f.write("path_length = "+str(ps.client.problem.pathLength(i)) + "\n")
  f.close()
  print "problem "+str(i)+" solved \n"
  ps.clearRoadmap()
"""
#ps.client.problem.prepareSolveStepByStep()

#ps.client.problem.finishSolveStepByStep()

q_far = q_init[::]
q_far[2] = -3
r(q_far)





"""
camera = [0.6293167471885681,
 -9.560577392578125,
 10.504343032836914,
 0.9323806762695312,
 0.36073973774909973,
 0.008668755181133747,
 0.02139890193939209]
r.client.gui.setCameraTransform(0,camera)
"""

"""
r.client.gui.removeFromGroup("rm",r.sceneName)
r.client.gui.removeFromGroup("rmstart",r.sceneName)
r.client.gui.removeFromGroup("rmgoal",r.sceneName)
for i in range(0,ps.numberNodes()):
  r.client.gui.removeFromGroup("vecRM"+str(i),r.sceneName)

"""
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.03
pp.displayVelocityPath(0)
r.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")


"""
# for seed 1486657707
ps.client.problem.extractPath(0,0,2.15) 

# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.03
pp.displayVelocityPath(1)
r.client.gui.setVisibility("path_1_root","ALWAYS_ON_TOP")
#display path
pp.speed=0.3
#pp (0)
"""
#display path with post-optimisation




"""
q_far = q_init[::]
q_far[2] = -3
r(q_far)
"""




