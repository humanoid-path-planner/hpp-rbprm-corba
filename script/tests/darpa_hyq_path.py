from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer
white=[1.0,1.0,1.0,1.0]
green=[0.23,0.75,0.2,0.5]
yellow=[0.85,0.75,0.15,1]
pink=[1,0.6,1,1]
orange=[1,0.42,0,1]
brown=[0.85,0.75,0.15,0.5]
blue = [0.0, 0.0, 0.8, 1.0]
grey = [0.7,0.7,0.7,1.0]
red = [0.8,0.0,0.0,1.0]

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'hyq_trunk_large_realist'
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
rbprmBuilder.boundSO3([-0.4,0.4,-0.5,0.5,-0.5,0.5])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(3)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)



q_init = [-2, 0, 0.63,1,0,0,0,0,0,1]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
#~ q_init [0:3] = [2, 0, 0.63]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
q_goal [0:3] = [3, 0, 0.63]; r (q_goal)

#~ ps.addPathOptimizer("GradientBased")
# ps.addPathOptimizer("RandomShortcut")
ps.selectPathPlanner("RRTdynamic")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)
r.loadObstacleModel (packageName, "darpa", "planning")
r(q_init)

ps.solve ()
#r.solveAndDisplay("rm",1,0.02)

#r.displayRoadmap("rm",0.005)
r.displayPathMap("rmPath",0,0.02)



from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

#~ rbprmBuilder.exportPath (r, ps.client.problem, 1, 0.1, "darpa_hyq_robust_20_path.txt")
#~ pp.fromFile("/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")
#~ 
#~ pp (2)
#~ pp (0)
pp.displayPath(0)
r.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")
pp (0)

#r.client.gui.removeFromGroup("rm",r.sceneName)
r.client.gui.removeFromGroup("rmPath",r.sceneName)
#r.client.gui.removeFromGroup("path_1_root",r.sceneName)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")

