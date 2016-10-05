from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

from hpp.corbaserver.rbprm.tools.cwc_trajectory import *

from hpp import Error as hpperr
from numpy import array


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


ps = ProblemSolver( fullBody )
r = Viewer (ps)

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
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "manipulability", 0.05, "_6_DOF", True)

#~ limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
						#~ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}, 
						#~ lLegId : {'file': "hrp2/LA_com.ineq", 'effector' : lHand}, 
						#~ rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand} }
limbsCOMConstraints = { rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
						lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}

#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "manipulability", 0.05, "_6_DOF", True)

 #~ 

#~ fullBody.client.basic.robot.setJointConfig('LARM_JOINT0',[1])
#~ fullBody.client.basic.robot.setJointConfig('RARM_JOINT0',[-1])

import quaternion as quat


def _getTransform(qEffector):
	q0 = quat.Quaternion(qEffector[3:7])
	rot = q0.toRotationMatrix() #compute rotation matrix local -> world
	p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
	rm=np.zeros((4,4))
	for k in range(0,3):
		for l in range(0,3):
			rm[k,l] = rot[k,l]
	for m in range(0,3):
		rm[m,3] = qEffector[m]
	rm[3,3] = 1
	return rm
				

def draw_cp(cid, limb, config):
	global r
	#~ posetc = fullBody.getEffectorPosition(limb, config)
	P, N = fullBody.computeContactForConfig(config, limb)
	fullBody.setCurrentConfig(config)
	effectorName = limbsCOMConstraints[limb]['effector']
	limbId = limb
	m = _getTransform(fullBody.getJointPosition(effectorName))
	scene = "qds"+limb+str(cid)
	r.client.gui.createScene(scene)
	for i in range(4):
		#~ pos = posetc[2*i]
		print array(P[i]+[1])
		print m.dot(array(P[i]+[1]))
		pos = m.dot(array(P[i]+[1]))[:3]
		print "pos", pos
		r.client.gui.addBox(scene+"/b"+str(i),0.01,0.01,0.01, [1,0,0,1])
		r.client.gui.applyConfiguration(scene+"/b"+str(i),pos.tolist()+[1,0,0,0])
		r.client.gui.refresh()	
	r.client.gui.addSceneToWindow(scene,0)


def fill_contact_points(limbs, config, config_pinocchio):
	res = {}
	res["q"] = config_pinocchio[:]
	res["contact_points"] = {}
	res["P"] = []
	res["N"] = []
	for limb in limbs:
		effector = limbsCOMConstraints[limb]['effector']
		#~ posetc = fullBody.getEffectorPosition(limb, config)
		P, N = fullBody.computeContactForConfig(q, limb)		
		#~ posetc = fullBody.getEffectorPosition(limb, config)
		res["contact_points"][effector] = {}
		#~ res["contact_points"][effector]["P"] = [p for i, p in enumerate (posetc) if (i%2 == 0)]
		res["contact_points"][effector]["P"] = P
		#~ res["P"] += [p for i, p in enumerate (posetc) if (i%2 == 0)]
		res["P"] += P
		#~ res["contact_points"][effector]["N"] = [n for i, n in enumerate (posetc) if (i%2 == 1)]
		res["contact_points"][effector]["N"] = N
		#~ res["N"] += [n for i, n in enumerate (posetc) if (i%2 == 1)]
		res["N"] += N
	return res

q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.getSampleConfig()
qs = []; qs_gepetto = []; states = []
limbs = [lLegId,rLegId] 
for _ in range(10):
	q = fullBody.generateGroundContact(limbs)
	q_gep = q[:]
	quat_end = q[4:7]
	q[6] = q[3]
	q[3:6] = quat_end
	qs.append(q)
	qs_gepetto.append(q_gep)
	states.append(fill_contact_points(limbs,q_gep,q))
	
from pickle import dump
f1=open("configs_feet_on_ground_static_eq", 'w+')
dump(states, f1)
f1.close()
