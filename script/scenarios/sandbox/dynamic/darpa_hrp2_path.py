## Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer
import time
import math
import omniORB.any



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
urdfName = 'hrp2_trunk_lowBody_flexible'
urdfNameRoms =  ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""


# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)



# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter([])
#rbprmBuilder.setAffordanceFilter('hrp2_rarm_rom', ['Lean'])
#rbprmBuilder.setAffordanceFilter('hrp2_larm_rom', ['Lean'])
rbprmBuilder.setAffordanceFilter('hrp2_lleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hrp2_rleg_rom', ['Support'])
vMax = 0.2;
aMax = 0.1;
extraDof = 6
MU = 0.5

rbprmBuilder.setJointBounds ("base_joint_xyz", [-1.2,1.5,-0.1 ,0.1, 0.55, 0.85])
rbprmBuilder.setJointBounds('CHEST_JOINT0',[-0.05,0.05])
rbprmBuilder.setJointBounds('CHEST_JOINT1',[-0.05,0.05])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-0,0,-0.1,0.1,-0.1,0.1])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,0,0,0,0,0,0])
indexECS = rbprmBuilder.getConfigSize() - rbprmBuilder.client.basic.robot.getDimensionExtraConfigSpace()


# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(aMax))
ps.client.problem.setParameter("aMaxZ",omniORB.any.to_any(10.))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(vMax))
ps.client.problem.setParameter("orientedPath",omniORB.any.to_any(0.))
ps.client.problem.setParameter("friction",omniORB.any.to_any(MU))
ps.client.problem.setParameter("sizeFootX",omniORB.any.to_any(0.24))
ps.client.problem.setParameter("sizeFootY",omniORB.any.to_any(0.14))


r = Viewer (ps,displayArrows = True)


from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.005])
afftool.loadObstacleModel (packageName, "darpa_no_mid", "planning", r,reduceSizes=[0.14,0])
#r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
r.addLandmark(r.sceneName,1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [1,0,0,0]
q_init [0:3] = [-0.95, 0, 0.53]; r (q_init)
#q_init [0:3] = [-0.5, 0, 0.75]; r (q_init)

rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]

q_goal [0:3] = [1.2, 0, 0.53]; r(q_goal)
#q_goal [0:3] = [1, 0, 0.75]; r(q_goal)
r (q_goal)


# Choosing a path optimizer
#ps.setInitialConfig (q_init)
#ps.addGoalConfig (q_goal)


q1=q_init[::]
q1[0:3] = [-0.3, 0, 0.77]
q2=q_goal[::]
q2[0:3] = [0.8, 0, 0.77]
ps.setInitialConfig (q1)
ps.addGoalConfig (q2)

# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.01)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
#ps.addPathOptimizer("RandomShortcutDynamic")
#ps.addPathOptimizer("OrientedPathOptimizer")
ps.selectPathPlanner("BiRRTPlanner")
#ps.selectPathPlanner("DynamicPlanner")

#solve the problem :
r(q_init)





ps.client.problem.prepareSolveStepByStep()

"""
q1=q_init[::]
q1[0:3] = [-0.45, 0, 0.75]
q2=q_goal[::]
q2[0:3] = [0.9, 0, 0.75]
pbCl = rbprmBuilder.client.basic.problem


pbCl.addConfigToRoadmap (q1)
pbCl.addConfigToRoadmap (q2)

pbCl.directPath(q_init,q1,True)
pbCl.directPath(q1,q2,True)
pbCl.directPath(q2,q_goal,False)


pbCl.addEdgeToRoadmap (q_init, q1, 0, False)
pbCl.addEdgeToRoadmap (q1, q2, 1, False)
pbCl.addEdgeToRoadmap (q2, q_goal, 2, False)
"""
ps.client.problem.finishSolveStepByStep()





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

q=[1.0220000000000002,
 -0.01,
 0.6955999999999999,
 1.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.3,
 0.0,
 -0.17000000000000023,
 0.0,
 0.0,
 0.0] # BUG




# Playing the computed path
pi = ps.numberPaths()-1
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.03
pp.displayVelocityPath(pi)
r.client.gui.setVisibility("path_"+str(pi)+"_root","ALWAYS_ON_TOP")
#display path
pp.speed=1
#pp (pi)

"""
import parse_bench

parse_bench.parseBenchmark(t)
"""
print("path lenght = ",str(ps.client.problem.pathLength(ps.numberPaths()-1)))

###########################
#display path with post-optimisation

"""
print("Press Enter to display the optimization ...")
raw_input()
i=0

r.client.gui.removeFromGroup("path_0_root",r.sceneName)
pp.displayVelocityPath(1)

for i in range(1,5):
  time.sleep(3)
  ps.optimizePath(i)
  r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName)
  pp.displayVelocityPath(i+1)
"""

###########################


"""
q_far = q_init[::]
q_far[2] = -3
r(q_far)
"""


"""
for i in range(1,10):
    rbprmBuilder.client.basic.problem.optimizePath(i)
    r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName)
    pp.displayVelocityPath(i+1)
    #time.sleep(2)
"""

"""
i=0

ps.clearRoadmap()
ps.solve()
r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName)
i = i+1
pp.displayVelocityPath(i)

pp(i)


"""

"""
r.client.gui.addCurve("c1",qlist,r.color.red)
r.client.gui.setVisibility("c1","ALWAYS_ON_TOP")
r.client.gui.addToGroup("c1",r.sceneName)


r.client.gui.addCurve("c2",qlist2,r.color.blue)
r.client.gui.setVisibility("c2","ALWAYS_ON_TOP")
r.client.gui.addToGroup("c2",r.sceneName)



"""


"""
nodes = ["hyq_trunk_large/base_link","Vec_Acceleration","Vec_Velocity"]
r.client.gui.setCaptureTransform("yaml/hyq_slalom_path.yaml",nodes)
r.client.gui.captureTransformOnRefresh(True)
pp(1)
r.client.gui.captureTransformOnRefresh(False)

r.client.gui.writeNodeFile('path_1_root','meshs/slalom_path.obj')

"""


