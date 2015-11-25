from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import darpa_hrp2_path as tp



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
fullBody.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])


ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)

#~ AFTER loading obstacles
rLegId = '0rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "forward", 0.03)

lLegId = '1lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "forward", 0.03)


q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, larmId, q_0,)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])
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




fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])
#~ 
#~ r(q_init)
configs = fullBody.interpolate(0.1)
r.loadObstacleModel ('hpp-rbprm-corba', "darpa", "contact")
#~ configs = fullBody.interpolate(0.09)
#~ configs = fullBody.interpolate(0.08)
i = 0; 
#~ r (configs[i]); i=i+1; i-1
#~ q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)
#~ fullBody.draw(q_0,r)
#~ fullBody.client.rbprm.rbprm.getOctreeTransform(larmId, q_0)
#~ problem = ps.client.problem
#~ length = problem.pathLength (0)
#~ t = 0
#~ i = 0
#~ configs = []
#~ dt = 0.1 / length
#~ while t < length :
	#~ q = fullBody.getCurrentConfig()
	#~ q[0:confsize] = problem.configAtParam (0, t)[0:confsize]
	#~ configs.append(q)
	#~ t += dt
	#~ i = i+1
	#~ 
i = 0;
fullBody.draw(configs[i],r); i=i+1; i-1
#~ 
#~ f1 = open("hrp2_standing_29_10_15","w+")
#~ f1.write(str(configs))
#~ f1.close()
