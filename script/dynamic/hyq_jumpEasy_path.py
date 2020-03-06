from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer


rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
# URDF file describing the trunk of the robot HyQ
urdfName = 'hyq_trunk'
# URDF files describing the reachable workspace of each limb of HyQ
urdfNameRom = ['hyq_lhleg_rom','hyq_lfleg_rom','hyq_rfleg_rom','hyq_rhleg_rom']
urdfSuffix = ""
srdfSuffix = ""
vMax = 4;
aMax = 10;
extraDof = 6
rbprmBuilder = Builder ()
rbprmBuilder.loadModel(urdfName, urdfNameRom, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-6,6, -3, 3, 0, 2.5])
rbprmBuilder.boundSO3([-0.1,0.1,-1,1,-1,1])
rbprmBuilder.setFilter(['hyq_rhleg_rom', 'hyq_lfleg_rom', 'hyq_rfleg_rom','hyq_lhleg_rom'])
rbprmBuilder.setAffordanceFilter('hyq_rhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_rfleg_rom', ['Support',])
rbprmBuilder.setAffordanceFilter('hyq_lhleg_rom', ['Support'])
rbprmBuilder.setAffordanceFilter('hyq_lfleg_rom', ['Support',])
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(extraDof)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([-vMax,vMax,-vMax,vMax,0,0,0,0,0,0,0,0])


#~ from hpp.corbaserver.rbprm. import ProblemSolver
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( rbprmBuilder )
ps.client.problem.setParameter("aMax",aMax)
ps.client.problem.setParameter("vMax",vMax)
ps.client.problem.setParameter("tryJump",1)
r = Viewer (ps)

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, "ground_jump_easy", "planning", r)
afftool.visualiseAffordances('Support', r, r.color.brown)
r.addLandmark(r.sceneName,1)
q_init = rbprmBuilder.getCurrentConfig ();
#q_init[(len(q_init)-3):]=[0,0,1] # set normal for init / goal config
#q_init [0:3] = [-3, 0, 0.79]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)
q_init [0:3] = [-3, 0, 0.7]; rbprmBuilder.setCurrentConfig (q_init); r (q_init)

q_goal = q_init [::]
#q_goal [0:3] = [-2, 0, 0.9]; r (q_goal) # premiere passerelle
q_goal [0:3] = [0.7, 0, 0.7]; r (q_goal) # pont
#q_goal = [-0.594272,0,0.724869,1,0,0,0,3.26968,0,1.85351,0,0,0]
#q_goal [0:3] = [-3, 0, 0.8]
#q_goal[7:10] = [0,0,2.2]


ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
# Choosing RBPRM shooter and path validation methods.
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.client.problem.selectPathValidation("RbprmPathValidation",0.01)
# Choosing kinodynamic methods : 
ps.selectSteeringMethod("RBPRMKinodynamic")
ps.selectDistance("KinodynamicDistance")
ps.selectPathPlanner("DynamicPlanner")



r(q_init)


ps.client.problem.prepareSolveStepByStep()

ps.client.problem.finishSolveStepByStep()

#i = 0
#r.displayRoadmap("rm"+str(i),0.02)
#ps.client.problem.executeOneStep() ;i = i+1; r.displayRoadmap("rm"+str(i),0.02) ; r.client.gui.removeFromGroup("rm"+str(i-1),r.sceneName) ;
#t = ps.solve ()



# Playing the computed path
from hpp.gepetto import PathPlayer
pp = PathPlayer (rbprmBuilder.client.basic, r)
pp.dt=1./30.
#r.client.gui.removeFromGroup("rm0",r.sceneName)
pp.displayVelocityPath(0)
pp.speed=0.2
pp(0)

"""

#r.client.gui.removeFromGroup("rm",r.sceneName)
r.client.gui.removeFromGroup("rmPath",r.sceneName)
r.client.gui.removeFromGroup("path_1_root",r.sceneName)
#~ pp.toFile(1, "/home/stonneau/dev/hpp/src/hpp-rbprm-corba/script/paths/stair.path")


math.sqrt((np.linalg.norm(u)*np.linalg.norm(u)) * (np.linalg.norm(v)*np.linalg.norm(v))

from hpp import quaternion as Quaternion
q = Quaternion.Quaternion([1,0,0],[2,3,-1])


"""

q_far = q_init[::]
q_far[2] = -3
r(q_far)




