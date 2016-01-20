from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hyq_trunk_large'
urdfNameRom = ['hyq_lhleg_rom','hyq_lfleg_rom','hyq_rfleg_rom','hyq_rhleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()

rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setNormalFilter('hyq_lhleg_rom', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('hyq_rfleg_rom', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('hyq_lfleg_rom', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('hyq_rhleg_rom', [0,0,1], 0.5)
rbprmBuilder.boundSO3([-0.4,0.4,-3,3,-3,3])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)


q_init = rbprmBuilder.getCurrentConfig ();
q_init [0:3] = [-2, 0, 0.63]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [2, 0, 0.63]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
q_goal [0:3] = [3, 0, 0.63]; r (q_goal)

#~ ps.addPathOptimizer("GradientBased")
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
r.loadObstacleModel (packageName, "darpa", "planning")
r(q_init)
#~ ps.solve ()
t = ps.solve ()
if isinstance(t, list):
  t = t[len(t)-1]

f = open('log.txt', 'a')
f.write("path computation " + str(t) + "\n")
f.close()
r.displayRoadmap("rm",white,0.01,1,green)


from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

#~ rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.1, "darpa_hyq_robust_20_path.txt")
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)

pp (1)

# r.client.gui.removeFromGroup("rm",r.sceneName)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")

