from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hrp2_trunk_flexible'
urdfNameRoms =  ['hrp2_larm_rom','hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [0,2, -1, 1, 0, 2.2])
#~ rbprmBuilder.setFilter(['hrp2_rarm_rom','hrp2_lleg_rom','hrp2_rleg_rom'])
#~ rbprmBuilder.setNormalFilter('hrp2_rarm_rom', [0,0,1], 0.5)
#~ rbprmBuilder.setNormalFilter('hrp2_lleg_rom', [0,0,1], 0.9)
#~ rbprmBuilder.setNormalFilter('hrp2_rleg_rom', [0,0,1], 0.9)
#~ rbprmBuilder.setNormalFilter('hyq_rhleg_rom', [0,0,1], 0.9)
rbprmBuilder.boundSO3([-0.,0,-1,1,-1,1])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [0, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
q_init [0:3] = [0.1, -0.82, 0.648702]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [0, -0.63, 0.6]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]

q_goal = q_init [::]
q_goal [3:7] = [ 0.98877108,  0.        ,  0.14943813,  0.        ]
q_goal [0:3] = [1.49, -0.65, 1.25]; r (q_goal)
#~ q_goal [0:3] = [1.2, -0.65, 1.1]; r (q_goal)

#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
r.loadObstacleModel (packageName, "stair_bauzil", "planning")
#~ ps.solve ()
t = ps.solve ()
f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()

print ("solving time " + str(t));

from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)

#~ pp (1)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.01, "stair_bauzil_hrp2_path.txt")
r (q_init)
