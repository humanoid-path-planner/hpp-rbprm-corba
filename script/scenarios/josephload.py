from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import josephpath as tp

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
fullBody.setJointBounds ("base_joint_xyz", [-1,2, -1, 1, 0, 2.2])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', "ground", "contact")

#~ AFTER loading obstacles
rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "manipulability", 0.05)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000,"manipulability", 0.03)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

q_0 = fullBody.getCurrentConfig(); 
q_init =  [
	0.0, 0.0, 0.648702, 1.0, 0.0, 0.0, 0.0,                         # Free flyer 0-6
	0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
	0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # LARM       11-17
	0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # RARM       18-24
	0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
	0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
	]; r (q_init)



q_init[0:7] = tp.q_init[0:7]
q_goal =  q_init [::]
q_goal[0:7] = tp.q_goal[0:7]
fullBody.setCurrentConfig (q_goal)

fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])
#~ 
#~ configs = fullBody.interpolate(0.1)
configs = fullBody.interpolate(0.1)
#~ configs = fullBody.interpolate(0.08)
i = 0; 
r (configs[i]); i=i+1; i-1
#~ q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)

