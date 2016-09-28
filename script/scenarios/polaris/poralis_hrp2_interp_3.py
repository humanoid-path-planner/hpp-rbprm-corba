from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer

import polaris_hrp2_path_3 as tp



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
fullBody.setJointBounds ("base_joint_xyz", [-1,2, -2, 1, 0.5, 2.5])

ps = tp.ProblemSolver( fullBody )
r = tp.Viewer (ps)

#~ AFTER loading obstacles
#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "static", 0.03)

lLegId = 'hrp2_lleg_rom'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "static", 0.03)


rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [-0.05,-0.050,-0.050]
rArmNormal = [1,0,0]
rArmx = 0.024; rArmy = 0.024
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 20000, "EFORT", 0.05, "_6_DOF", True)

larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 20000, "static", 0.05, "_6_DOF", True)

 #~ 
fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(larmId, "jointLimitsDistance", True)

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, larmId, q_0,)

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])
confsize = len(tp.q_init)
        
q_init = [0.36,
 -0.9940043559354331,
 0.9006595208471023,
 1.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 0.0,
 -0.07181393332784843,
 0.7655372968657125,
 0.2121567845892829,
 -1.6017728439499854,
 0.3498201890812377,
 0.1893419005076257,
 0.17,
 -0.2127190633199182,
 0.12167821044167787,
 0.3231549720640773,
 -1.5730826945359044,
 -0.22989879775081978,
 0.31616997548686604,
 0.17,
 0.7594103448503245,
 -0.17485863012578112,
 -0.9656332656586277,
 1.5895095478884926,
 -0.6238359283780285,
 0.17483187273314665,
 -0.687293355607262,
 -0.11841270055084166,
 -0.7903724619230399,
 1.5860588209778248,
 -0.7956764813669736,
 0.11846562142112675]

     

r(q_init)

q_goal = fullBody.getCurrentConfig(); q_goal[0:confsize] = tp.q_goal[0:confsize]


fullBody.setCurrentConfig (q_init)
#~ q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.generateContacts(q_init, [0,0,-1]); r (q_init)

fullBody.setCurrentConfig (q_goal)
#~ r(q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])
#~ r(q_goal)

#~ gui = r.client.gui




#~ fullBody.setStartState(q_init,[rLegId,lLegId,rarmId,larmId])
fullBody.setStartState(q_init,[rLegId,lLegId,larmId])
fullBody.setEndState(q_goal,[rLegId,lLegId])
#~ 
#~ r(q_init)
configs = fullBody.interpolate(0.01, 10, 2, True)
#~ r.loadObstacleModel ('hpp-rbprm-corba', "polaris", "contact")
#~ configs = fullBody.interpolate(0.09)
#~ configs = fullBody.interpolate(0.08)
i = 0; 
r (configs[i]); i=i+1; i-1
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

import datetime
now = datetime.datetime.now()
#~ 
f1 = open("polaris_hrp2"+str(now),"w+")
f1.write(str(configs))
f1.close()

#~ import hpp.gepetto.blender.exportmotion as em

#~ fullBody.exportAll(r, configs, "polaris_hrp2_OK"+str(now));
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj

	
limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'},
						rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand},
						larmId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand} }

#~ fullBody.limbRRTFromRootPath(0,len(configs)-1,0,2)
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

def act(i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, verbose = False, draw = False, timescale = 20.):
	return step(fullBody, configs, i, numOptim, pp, limbsCOMConstraints, 0.4, optim_effectors = optim_effectors, time_scale =timescale, useCOMConstraints = True, use_window = use_window, verbose = verbose, draw = draw)

def play(frame_rate = 1./24.):
	play_traj(fullBody,pp,frame_rate)
	
def saveAll(name):
	saveAllData(fullBody, r, name)

#~ for i in range(0,20):  act(i,numOptim = 0, use_window = 0, friction = 1, optim_effectors = False, verbose = False, draw = False)

#~ for i in range(10,30):  act(i,numOptim = 60, use_window = 0, friction = 1, optim_effectors = True, verbose = False, draw = False)
