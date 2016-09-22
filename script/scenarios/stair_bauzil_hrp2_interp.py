from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import stair_bauzil_hrp2_path as tp



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
fullBody.setJointBounds ("base_joint_xyz", [-0.135,2, -1, 1, 0, 2.2])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)

#~ AFTER loading obstacles
rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.1)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.1)

rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1]
rArmNormal = [0,0,1]
rArmx = 0.024; rArmy = 0.024
#disabling collision for hook
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.05, "_6_DOF", True)


#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, 0.05)

rKneeId = '0RKnee'
rLeg = 'RLEG_JOINT0'
rKnee = 'RLEG_JOINT3'
rLegOffset = [0.105,0.055,0.017]
rLegNormal = [-1,0,0]
rLegx = 0.05; rLegy = 0.05
#~ fullBody.addLimb(rKneeId, rLeg,rKnee,rLegOffset,rLegNormal, rLegx, rLegy, 10000, 0.01)
#~ 
lKneeId = '1LKnee'
lLeg = 'LLEG_JOINT0'
lKnee = 'LLEG_JOINT3'
lLegOffset = [0.105,0.055,0.017]
lLegNormal = [-1,0,0]
lLegx = 0.05; lLegy = 0.05
#~ fullBody.addLimb(lKneeId,lLeg,lKnee,lLegOffset,lLegNormal, lLegx, lLegy, 10000, 0.01)
 #~ 

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]


fullBody.setCurrentConfig (q_init)
# Free flyer 0-6 / CHEST HEAD 7-10 / LARM 11-17 / RARM 18-24 / LLEG 25-30 / RLEG 31-36

q_init =  [0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0, 0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0]; r (q_init)

fullBody.setCurrentConfig (q_goal)
#~ r(q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
#~ r(q_goal)

fullBody.setStartState(q_init,[rLegId,lLegId]) #,rarmId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])#,rarmId,larmId])
#~ 
#~ configs = fullBody.interpolate(0.1)
configs = fullBody.interpolate(0.1)
#~ configs = fullBody.interpolate(0.15)
i = 0;
fullBody.draw(configs[i],r); i=i+1; i-1

r.loadObstacleModel ('hpp-rbprm-corba', "stair_bauzil", "contact")
#~ fullBody.exportAll(r, configs, 'stair_bauzil_hrp2_robust_2');
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT0',[-1])
#~ q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.draw(q_0,r);
#~ print(fullBody.client.rbprm.rbprm.getOctreeTransform(rarmId, q_0))
#~ 
#~ 
#~ fullBody.client.basic.robot.setJointConfig('LLEG_JOINT0',[1])
#~ q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.draw(q_0,r);
#~ print(fullBody.client.rbprm.rbprm.getOctreeTransform(rarmId, q_0))
#~ q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)

#~ f1 = open("secondchoice","w+")
#~ f1 = open("hrp2_stair_not_robust_configs","w+")
#~ f1.write(str(configs))
#~ f1.close()

from hpp.gepetto import PathPlayer
#~ pp = PathPlayer (ps.robot.client.basic, r, 0.001, 0.1)
pp = PathPlayer (ps.robot.client.basic, r)
#~ (self, client, publisher, dt=0.01, speed=1)

