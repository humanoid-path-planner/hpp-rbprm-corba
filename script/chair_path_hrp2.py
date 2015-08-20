from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk'
urdfNameRoms = ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-1, 1, -0.5, 0.5, 0, 5])
rbprmBuilder.setFilter(['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom'])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

q_init = rbprmBuilder.getCurrentConfig (); r (q_init)
q_goal =  rbprmBuilder.getCurrentConfig ();
#~ rbprmBuilder.client.basic.robot.setJointConfig('base_joint_SO3',[0.7316888688738209, 0, -0.6816387600233341, 0]); q_init = rbprmBuilder.getCurrentConfig (); r (q_init)

q_init = rbprmBuilder.getCurrentConfig ();
#~ q_init = [-0.15, -0.2, 0.6, 0.99500416527802582, 0.0, 0.099833416646828155, 0.0]
q_init = [-0.3, -0.2, 0.6, 1, 0.0, 0., 0.0]; r (q_init)
rbprmBuilder.setCurrentConfig (q_init); r (q_init)

#~ q_goal = [0, 0, 2, 0.99500416527802582, 0.0, 0.099833416646828155, 0.0]; r (q_goal)
#~ q_goal = [0.2, -0.2, 1.6, 0.99500416527802582, 0.0, 0.099833416646828155, 0.0]; r (q_goal)
q_goal = [0.1, -0.2, 1.6, 0.98877107793604235, 0.0, 0.14943813247359924, 0.0]
ps.addPathOptimizer("RandomShortcut")
#~ ps.addPathOptimizer("GradientBased")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.1)
r.loadObstacleModel (packageName, "chair", "planning")
ps.solve ()


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

#~ pp (0)
pp (1)
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/truck.path")
#~ pp (1)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/truck.path")

