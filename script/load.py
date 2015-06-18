from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'box'
urdfNameRom = 'box_rom'
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [0, 2, -2, 0, 0, 1.5])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

q_init = rbprmBuilder.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [0, -0.5, 0.4]

#~ rank = rbprmBuilder.rankInConfiguration ['torso_lift_joint']
#~ q_init [rank] = 0.2
r (q_init)

q_goal [0:3] = [1, -0.5, 1]
#~ q_goal [0:3] = [-3.2, 0, 3]
#~ rank = rbprmBuilder.rankInConfiguration ['l_shoulder_lift_joint']
#~ q_goal [rank] = 0.5
#~ rank = rbprmBuilder.rankInConfiguration ['l_elbow_flex_joint']
#~ q_goal [rank] = -0.5
#~ rank = rbprmBuilder.rankInConfiguration ['r_shoulder_lift_joint']
#~ q_goal [rank] = 0.5
#~ rank = rbprmBuilder.rankInConfiguration ['r_elbow_flex_joint']
#~ q_goal [rank] = -0.5
r (q_goal)

r.loadObstacleModel (packageName, "scene", "car")

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")

ps.solve ()


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

pp (0)

