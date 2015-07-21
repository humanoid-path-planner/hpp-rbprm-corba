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
#~ 
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
fullBody.addLimb(rLeg,rLegOffset, 5000, 0.001)

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
fullBody.addLimb(lLeg,lLegOffset, 10000, 0.001)

fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])
fullBody.client.basic.robot.setJointConfig('LLEG_JOINT3',[1.5])
fullBody.client.basic.robot.setJointConfig('RLEG_JOINT3',[1.5])


q_init = fullBody.getCurrentConfig ()
q_init [0:3] = [0, -0.5, 0.6]
r (q_init)

q_goal = q_init [::]
q_goal [0:3] = [0.5, -0.5, 0.7]

fullBody.setCurrentConfig (q_init)

q_init = fullBody.generateContacts(q_init, [0,0,1])
r (q_init)
#~ fullBody.getContactSamplesIds(rLeg, q_init, [0,0,1])
#~ q_init = fullBody.getSample(rLeg, 1)

#~ ids = fullBody.getContactSamplesIds(rLeg, q_init, [0,0,1])
#~ i =-1
#~ i=i+1; q_init = fullBody.getSample(rLeg, int(ids[i])); r(q_init); ids[i]
#~ 
#~ fullBody.setCurrentConfig (q_goal)
#~ q_goal = fullBody.generateContacts(q_goal, [0,0,1])

#~ ps.setInitialConfig (q_init)
#~ ps.addGoalConfig (q_goal)
#~ ps.solve ()

#~ 
#~ fullBody.setStartState(q_init,[rLeg,lLeg])
#~ fullBody.setEndState(q_goal,[rLeg])

#~ configs = fullBody.interpolate(0.1)
#~ i = 0; 
#~ r (configs[i]); i=i+1; i

