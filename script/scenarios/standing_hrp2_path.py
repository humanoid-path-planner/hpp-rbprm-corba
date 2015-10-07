from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms = ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-1,1, -1, 1, 0, 2.2])
rbprmBuilder.setFilter(['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom'])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

q_init = rbprmBuilder.getCurrentConfig (); r (q_init)
q_goal = q_init [::]
q_goal [0:3] = [0.19, 0.05, 0.9]; r (q_goal)

#~ fullBody.client.basic.robot.setJointConfig('base_joint_SO3',[0.7316888688738209, 0, -0.6816387600233341, 0]); q_init = rbprmBuilder.getCurrentConfig (); r (q_init)

rbprmBuilder.client.basic.robot.setJointConfig('base_joint_SO3',[0.7316888688738209, 0, 0.6816387600233341, 0]); q_init = rbprmBuilder.getCurrentConfig (); r (q_init)
q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-0.8, 0.05, 0.5]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_0 [0:3] = [-0.2, 0, 0.3]; r (q_0)


q_goal [0:3] = [0.13, 0.05, 0.8]; r (q_goal)
#~ q_init [0:6] = [0.0, -2.2, 2.0, 0.7316888688738209, 0.0, 0.6816387600233341]; 
#~ rbprmBuilder.setCurrentConfig (q_init); r (q_init)


#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

#~ ps.client.problem.selectConFigurationShooter("RbprmShooter")
#~ ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
r.loadObstacleModel (packageName, "scene_wall", "planning")
ps.solve ()


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)

pp (1)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
r (q_init)
