## Importing helper class for setting up a reachability planning problem
from hpp.corbaserver.rbprm.rbprmbuilder import Builder

# Importing Gepetto viewer helper class
from hpp.gepetto import Viewer
import time

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
# URDF file describing the trunk of the robot HyQ
urdfName = 'hyq_trunk_large'
# URDF files describing the reachable workspace of each limb of HyQ
urdfNameRom = ['hyq_lhleg_rom','hyq_lfleg_rom','hyq_rfleg_rom','hyq_rhleg_rom']
urdfSuffix = ""
srdfSuffix = ""
vMax = 4;
aMax = 5;
extraDof = 6
# Creating an instance of the helper class, and loading the robot
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
#rbprmBuilder.setJointBounds ("base_joint_xyz", [-1.25,2, -0.5, 5.5, 0.6, 1.8])
rbprmBuilder.setJointBounds ("base_joint_xyz", [-1.25,5, 0, 2, 0.45, 1.8])
# The following lines set constraint on the valid configurations:
# a configuration is valid only if all limbs can create a contact ...
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support',])
# We also bound the rotations of the torso. (z, y, x)
rbprmBuilder.boundSO3([-0.1,0.1,-0.65,0.65,-0.2,0.2])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-vMax,vMax,-1,1,-2,2,0,0,0,0,0,0])

# Creating an instance of HPP problem solver and the viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",aMax)
ps.client.problem.setParameter("vMax",vMax)
r = Viewer (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [0.5, 0.03, 0.00005])
afftool.loadObstacleModel (packageName, "downSlope", "planning", r)
#r.loadObstacleModel (packageName, "ground", "planning")
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])
r.addLandmark(r.sceneName,1)

# Setting initial and goal configurations
q_init = rbprmBuilder.getCurrentConfig ();
q_init[3:7] = [0.9659,0,0.2588,0]
q_init [0:3] = [-1.25, 1, 1.7]; r (q_init)
#q_init[3:7] = [0.7071,0,0,0.7071]
#q_init [0:3] = [1, 1, 0.65]

rbprmBuilder.setCurrentConfig (q_init)
q_goal = q_init [::]
#q_goal[3:7] = [0.7071,0,0,0.7071]
#q_goal [0:3] = [1, 5, 0.65]; r(q_goal)
q_goal[3:7] = [1,0,0,0]
q_goal [0:3] = [2, 1, 0.60]; r(q_goal)
#q_goal[3:7] = [0.9659,0,0.2588,0]
#q_goal[7:10] = [vMax,0,-2]
#q_goal [0:3] = [0, 1, 0.8]; r(q_goal)

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

#ps.client.problem.prepareSolveStepByStep()

q_far = q_init[::]
q_far[2] = -3
r(q_far)

#r.solveAndDisplay("rm",1,0.01)


ps.solve ()

# seed = 1486546394 (hyq_trunk, 2m)

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


# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=0.03
pp.displayVelocityPath(0)
r.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
#display path
pp.speed=0.3
#pp (0)

#display path with post-optimisation


ps.client.problem.extractPath(0,0,1.95)
r.client.gui.removeFromGroup("path_0_root",r.sceneName)
pp.displayVelocityPath(1)
#pp (1)



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



