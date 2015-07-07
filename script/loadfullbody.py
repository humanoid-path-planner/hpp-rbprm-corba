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

rbprmBuilder.loadFullBodyModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [0, 2, -2, 0, -1, 1.5])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

q_init = rbprmBuilder.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:3] = [0, -0.5, 0.3]
q_goal = [0, -0.5, -0.2, -0.501544,0.431183, 0.662926, -0.350804]
r (q_goal)


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

