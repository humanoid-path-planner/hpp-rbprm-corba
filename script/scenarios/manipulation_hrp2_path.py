# vim: foldmethod=marker foldlevel=2
from hpp.corbaserver.manipulation.hrp2 import Robot
from hpp.corbaserver.manipulation import ProblemSolver as ManipulationProblemSolver, ConstraintGraph, Client as ManipClient
from hpp.gepetto.manipulation import ViewerFactory as ManipulationViewerFactory
from hpp.gepetto import PathPlayer, PathPlayerGui

# Create a new manipulation problem
cl = ManipClient()
cl.problem.selectProblem("manipulationProblem")

class Box (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_tutorial'
  urdfName = 'box'
  urdfSuffix = ""
  srdfSuffix = ""

# Robot.urdfSuffix = '_capsule_mesh'
Robot.urdfSuffix = ''
Robot.srdfSuffix = '_manipulation'

# Load HRP2 and a box {{{3
robot = Robot ('hrp2-box', 'hrp2')
ps = ManipulationProblemSolver (robot)
ps.selectPathPlanner ("M-RRT")
vf = ManipulationViewerFactory (ps)
vf.loadObjectModel (Box, 'box')
for d in ["hrp2", "box"]:
  robot.setJointBounds (d+"/base_joint_xyz", [-4,4,-4,4,-4,4])

ps.selectPathProjector ("Progressive", 0.2)
ps.setErrorThreshold (1e-3)
ps.setMaxIterations (40)
# 3}}}

# Define configurations {{{3
half_sitting = q = robot.getInitialConfig ()
q_init = half_sitting [::]
# Set initial position of screw-driver
q_init [-7:] = [2, 1, 0.65, 1, 0, 0, 0]
# q_init [-7:] = [2, 1, 0.65, 0.7071067811865476, 0.5, -0.5, 0]
# Open left hand
ilh = robot.rankInConfiguration ['hrp2/LARM_JOINT6']
q_init [ilh:ilh+6] = [0.75, -0.75, 0.75, -0.75, 0.75, -0.75]

q_goal = q_init [::]
q_goal [-7:] = [2, -1, 0.65, 1, 0, 0, 0]

# 3}}}

# Generate constraints {{{3
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['hrp2'] = list ()
for n in jointNames['all']:
  if n.startswith ("hrp2"):
    jointNames['hrp2'].append (n)

ps.addPassiveDofs ('hrp2', jointNames ['hrp2'])

lockscrewgun = ps.lockFreeFlyerJoint ('box/base_joint', 'screwgun_lock')

locklhand = ['larm_6','lhand_0','lhand_1','lhand_2','lhand_3','lhand_4']
ps.createLockedJoint ('larm_6' , 'hrp2/LARM_JOINT6' , [q_init[ilh],])
ps.createLockedJoint \
    ('lhand_0', 'hrp2/LHAND_JOINT0', [q_init[ilh + 1],])
ps.createLockedJoint \
    ('lhand_1', 'hrp2/LHAND_JOINT1', [q_init[ilh + 2],])
ps.createLockedJoint \
    ('lhand_2', 'hrp2/LHAND_JOINT2', [q_init[ilh + 3],])
ps.createLockedJoint \
    ('lhand_3', 'hrp2/LHAND_JOINT3', [q_init[ilh + 4],])
ps.createLockedJoint \
    ('lhand_4', 'hrp2/LHAND_JOINT4', [q_init[ilh + 5],])

ps.createOrientationConstraint ("hrp2/waist_ori", robot.waist, "", (1,0,0,0), (True, True, False))

ps.addPartialCom ('hrp2', ['hrp2/base_joint_xyz'])
ps.createStaticStabilityConstraints ("balance-hrp2", q_init, 'hrp2')
# 3}}}

# Make a graph
graph = ConstraintGraph.buildGenericGraph (robot, "graph",
        ['hrp2/leftHand'], ['box'], [['box/handle']], [[]],
        [], [])

graph.setConstraints (graph=True, lockDof = locklhand, numConstraints=ps.balanceConstraints () + ["hrp2/waist_ori"])

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

t = ps.solve()
print t

# r = vf.createRealClient()
# pp = PathPlayer (robot.client.basic, r)
