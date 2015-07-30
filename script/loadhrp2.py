from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-5, 5, -5, 5, -5, 5])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

ps = ProblemSolver( fullBody )
r = Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', "scene", "car")

#~ AFTER loading obstacles
rLegId = '1rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 20000, 0.01)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 20000, 0.01)

fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])
fullBody.client.basic.robot.setJointConfig('LLEG_JOINT3',[1.5])
fullBody.client.basic.robot.setJointConfig('RLEG_JOINT3',[1.5])


q_init = fullBody.getCurrentConfig ()
q_init [0:3] = [0, -0.5, 0.6]
r (q_init)

q_goal = q_init [::]
q_goal [0:3] = [1, -0.5, 0.6]

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
r(q_goal)
ps.solve ()

fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,-1])
r (q_init)

fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,-1])


from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)
#~ 
pp (0)

fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId])

configs = fullBody.interpolate(0.1)
i = 1; 
r (configs[i]); i=i+1; i-1
#~ pp.toFile(0, "/home/stonneau/test.txt")

