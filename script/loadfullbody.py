from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'box'
urdfNameRom = 'box_rom'
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0, 2, -2, 0, -1, 1.5])
fullBody.addLimb("base_joint_SO3", 100, 0.1)

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( fullBody )

r = Viewer (ps)

fullBody.getSample('base_joint_SO3', 1)
fullBody.client.rbprm.rbprm.getSampleConfig('base_joint_SO3', 1)
q_init = fullBody.client.rbprm.rbprm.getSampleConfig('base_joint_SO3', 1)
r (q_init)


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

