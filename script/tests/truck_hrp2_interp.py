from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time 

import truck_hrp2_path as tp



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
fullBody.setJointBounds ("base_joint_xyz", [0,2.2, -1, 1, 0.7, 2.5])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)
r.client.gui.removeFromGroup("planning",r.sceneName)
r.client.gui.removeFromGroup("hrp2_trunk_flexible",r.sceneName)

#~ AFTER loading obstacles
rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.03)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.03)

rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [-0.05,-0.050,-0.050]
rArmNormal = [1,0,0]
rArmx = 0.024; rArmy = 0.024
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 20000, "forward", 0.05)

larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 20000, "forward", 0.05)

q_0 = fullBody.getCurrentConfig(); 

confsize = len(tp.q_init)
q_init = fullBody.getCurrentConfig(); q_init[0:confsize] = tp.q_init[0:confsize]
q_goal = fullBody.getCurrentConfig(); q_goal[0:confsize] = tp.q_goal[0:confsize]


fullBody.setCurrentConfig (q_init)
#~ q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)

fullBody.setCurrentConfig (q_goal)
#~ r(q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
#~ r(q_goal)

#~ gui = r.client.gui




fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,rarmId])
#~ fullBody.setStartState(q_init,[rLegId,lLegId,larmId])
#~ fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])
#~ fullBody.setEndState(q_goal,[rLegId,lLegIdlarmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])
#~ 
#~ r(q_init)
configs = fullBody.interpolate(0.1)
r.loadObstacleModel ('hpp-rbprm-corba', "truck", "contact")
#~ fullBody.exportAll(r, configs, 'truck_hrp2_not_robust');


i = 0;
while(i < (len(configs)-1)):
  fullBody.draw(configs[i],r); i=i+1; i-1
  time.sleep(0.3)




#~ 
#~ f1 = open("hrp2_standing_29_10_15","w+")
#~ f1.write(str(configs))
#~ f1.close()
