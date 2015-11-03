# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver import Client, ProblemSolver
from hpp.corbaserver.wholebody_step import Client as WSClient
from hpp.corbaserver.wholebody_step import Problem
from hpp.gepetto import Viewer


from hpp.corbaserver.hrp2 import Robot
from hpp.gepetto import ViewerFactory

Robot.urdfSuffix = '_reduced'
Robot.srdfSuffix = ''

# Define which problems are to be solved
walk_forward=True
walk_sideway=True
walk_oblique=True
walk_various=True

# Load HRP2 and a screwgun {{{3
robot = Robot ('hrp2')
ps = ProblemSolver (robot)
#~ ps.selectPathPlanner ("DiffusingPlanner")
# ps.addPathOptimizer ("SmallSteps")
vf = ViewerFactory (ps)
robot.setJointBounds ("base_joint_xyz", [-4,4,-4,4,-4,4])
#~ vf.loadObstacleModel ("hpp_tutorial", "box", "box")
#~ ps.moveObstacle ("box/base_link_0", [0.5,0.415,0.7,1,0,0,0])

wcl = WSClient ()

ps.setErrorThreshold (1e-3)
ps.setMaxIterations (60)
# 3}}}

# Define configurations {{{3
half_sitting = q = robot.getInitialConfig ()
q_init = half_sitting [::]

print (q_init)

q_init =  [0.0, 0.0, 0.648702, 1.0, 0.0, 0.0, 0.0,
 0.0, 0.0, 0.0, 0.0, 
 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532925,
  #~ 0.0, 0.0, 0.0, 0.0, 0.0, 
 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532925,
  #~ 0.0, 0.0, 0.0, 0.0, 0.0,
 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0,
 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q_goal = q_init [::]
q_goal [0:2] = [1.05,0]
# 3}}}

# Generate constraints {{{3
constraintName = "balance"
wcl.problem.addStaticStabilityConstraints (constraintName, q_init, robot.leftAnkle,
                                           robot.rightAnkle, "",
                                           Problem.SLIDING)
balanceConstraints = [constraintName + "/relative-com",
                      constraintName + "/relative-orientation",
                      constraintName + "/relative-position",
                      constraintName + "/orientation-left-foot",
                      constraintName + "/position-left-foot"]
# 3}}}

ps.setNumericalConstraints ("balance", balanceConstraints);

ps.setInitialConfig (q_init)

r = Viewer (ps)
r(q_init)
### Walk forward
if walk_forward:
  ps.addGoalConfig (q_goal)
  ps.solve ()
  print "Solved forward"

### Walk sideway
q_goal_side = q_init[:]
q_goal_side [0:2] = [0, 0.95]
if walk_sideway:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal_side)
  ps.solve ()
  print "Solved sideway"

### Walk in oblique direction
q_goal_od = q_init[:]
q_goal_od [0:2] = [1.05, -0.95]
if walk_oblique:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal_od)
  ps.solve ()
  print "Solved oblique"

### Walk along a non-direct path
q_goal_ndp = q_init[:]
q_goal_ndp [0:2] = [1.05, 0.95]
if walk_various:
  ps.resetGoalConfigs ()
  ps.addGoalConfig (q_goal_ndp)
  ps.solve ()
  print "Solved various"
  
from hpp.gepetto import PathPlayer
pp = PathPlayer (robot.client, r)

pp(0)
pp(1)
pp(2)
pp(3)
