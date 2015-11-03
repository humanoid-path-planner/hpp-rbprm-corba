from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms = ['hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-1,2, -1, 1, 0, 2.2])
rbprmBuilder.setFilter(['hrp2_lleg_rom','hrp2_rleg_rom'])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

# Define which problems are to be solved
walk_forward=False
walk_sideway=True
walk_oblique=False
walk_various=False

q_init = rbprmBuilder.getCurrentConfig (); r (q_init)
q_init[0:7] =  [0.0, 0.0, 0.648702, 1.0, 0.0, 0.0, 0.0]; r (q_init)
q_goal = q_init [::]
q_goal [0:2] = [1.05,0]
ps.setInitialConfig (q_init)


r.loadObstacleModel (packageName, "ground", "planning")

ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)

### Walk forward
if walk_forward:
  ps.addGoalConfig (q_goal)
  ps.solve ()
  print "Solved forward"

### Walk sideway
q_goal = q_init[:]
q_goal [0:2] = [0, 0.95]
if walk_sideway:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal)
  ps.solve ()
  print "Solved sideway"

### Walk in oblique direction
q_goal = q_init[:]
q_goal [0:2] = [1.05, -0.95]
if walk_oblique:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal)
  ps.solve ()
  print "Solved oblique"

### Walk along a non-direct path
q_goal = q_init[:]
q_goal [0:2] = [1.05, 0.95]
if walk_various:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal)
  ps.solve ()
  print "Solved various"

from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp (0)
r (q_init)
