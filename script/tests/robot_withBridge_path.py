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
black=[0,0,0,1]

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'robot_test_trunk'
urdfNameRom = ['robot_test_lleg_rom','robot_test_rleg_rom']
urdfSuffix = ""
srdfSuffix = ""

rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-6,6, -3, 3, 0, 2.5])
# limits zyx
rbprmBuilder.boundSO3([-1,1,-0.2,0.2,-0.2,0.2])
rbprmBuilder.setFilter(['robot_test_lleg_rom', 'robot_test_rleg_rom'])
rbprmBuilder.setNormalFilter('robot_test_lleg_rom', [0,0,1], 0.5)
rbprmBuilder.setNormalFilter('robot_test_rleg_rom', [0,0,1], 0.5)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(3)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0])

#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )

r = Viewer (ps)

r.loadObstacleModel (packageName, "ground_jump_easy", "planning")



q_init = rbprmBuilder.getCurrentConfig ();
q_init = [-4,1,0.9,1,0,0,0,0,0,1]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)


q_goal = q_init [::]
#q_goal [0:3] = [-2, 1, 0.6]; r (q_goal) # tryDirectPath
q_goal= [4,-1,0.9,1,0,0,0,0,0,1]; r (q_goal) 


#~ ps.addPathOptimizer("GradientBased")
#ps.addPathOptimizer("RandomShortcut")
#ps.client.problem.selectSteeringMethod("SteeringParabola")
ps.selectPathPlanner("RRTdynamic")
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05)

r(q_init)
#ps.solve()


r.solveAndDisplay("rm",1,0.02)

#rbprmBuilder.isConfigValid([-4,1,1.9,1,0,0,0,0,0,1])

#t = ps.solve ()
#r.displayRoadmap("rm",0.005)

#####
i = 0
ps.client.problem.prepareSolveStepByStep()
r.displayRoadmap("rm"+str(i),0.02)
ps.client.problem.executeOneStep() ;i = i+1; r.displayRoadmap("rm"+str(i),0.02) ; r.client.gui.removeFromGroup("rm"+str(i-1),r.sceneName); r(ps.node(ps.numberNodes()-1));


r.displayPathMap("rmPath",0,0.025)



from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)

pp.displayPath(0,r.color.lightGreen)

pp(0)

r.client.gui.setVisibility("path_0_root","ALWAYS_ON_TOP")

pp.displayPath(1,black)
pp (1)

#r.client.gui.removeFromGroup("rm",r.sceneName)
r.client.gui.removeFromGroup("rmPath",r.sceneName)
r.client.gui.removeFromGroup("path_0_root",r.sceneName)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")

i=0

ps.clearRoadmap(); ps.solve(); r.client.gui.removeFromGroup("path_"+str(i)+"_root",r.sceneName); pp.displayPath(i+1,r.color.lightGreen); i=i+1;






###############################


cu=[[-0.379915,0.75376,-0.0935008],[-0.455828,0.116297,-0.0891179],[-0.324647,-0.161917,0.195276],[-0.41015,-3.40644,-0.0576228],[-0.469389,0.0748034,0.572076],[-0.455828,0.116297,-0.0891179],[-0.469389,0.0748034,0.572076],[-0.455828,0.116297,-0.0891179],[-0.469389,0.0748034,0.572076],[-0.469389,0.0748034,0.572076]]
r.client.gui.addCurve("c2",cu,blue)
r.client.gui.addToGroup("c2",r.sceneName)
r.client.gui.setVisibility("c2","ALWAYS_ON_TOP")


