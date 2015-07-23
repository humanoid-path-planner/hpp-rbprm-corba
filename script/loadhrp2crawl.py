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
rLeg = 'RLEG_JOINT0'
rKnee = 'RLEG_JOINT3'
rLegOffset = [0.105,0.055,-0.017]
rLegNormal = [1,0,0]
rLegx = 0.05; rLegy = 0.05
fullBody.addLimb(rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 5000, 0.01)

lLeg = 'LLEG_JOINT0'
lKnee = 'RLEG_JOINT0'
lLegOffset = [0.105,0.055,-0.017]
lLegNormal = [1,0,0]
lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lLeg,lKnee,lLegOffset,rLegNormal, lLegx, lLegy, 5000, 0.01)

fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])
fullBody.client.basic.robot.setJointConfig('LLEG_JOINT3',[1.5])
fullBody.client.basic.robot.setJointConfig('RLEG_JOINT3',[1.5])


q_init = fullBody.getCurrentConfig ()
q_init [0:3] = [0, -0.5, 0.6]
r (q_init)

q_goal = q_init [::]
q_goal [0:3] = [1, -0.5, 0.6]

#~ 
#~ fullBody.setCurrentConfig (q_init)
#~ q_init = fullBody.generateContacts(q_init, [0,0,-1])
#~ r (q_init)

