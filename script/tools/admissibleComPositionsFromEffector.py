from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""

fullBody = FullBody ()
#~ fullBody.setJointBounds ("base_joint_xyz", [-2,2, -2, 2, -2, 2])
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-20,20, -20, 20, -20, 20])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 1

ps = ProblemSolver( fullBody )

rootName = 'base_joint_xyz'


rLegId = 'RL'
rLeg = 'RLEG_JOINT0'
rfoot = 'RLEG_JOINT5'
rLegOffset = [0,0,-0.105]
rLegNormal = [0,0,1]       
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "manipulability", 0.1)

lLegId = 'LL'
lLeg = 'LLEG_JOINT0'
lfoot = 'LLEG_JOINT5'                                                              
lLegx = 0.09; lLegy = 0.05      
lLegOffset = [0,0,-0.105]
lLegNormal = [0,0,1]          
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, "manipulability", 0.1)

rarmId = 'RA'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [0,0,-0.1075]
rArmNormal = [0,0,1]
rArmx = 0.024; rArmy = 0.024
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "manipulability", 0.05)

larmId = 'LA'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [0,0,-0.1075]
lArmNormal = [0,0,1]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "manipulability", 0.05)

#~ rLegId = 'rLeg'
#~ rLeg = 'RLEG_JOINT0'
#~ rfoot = 'RLEG_JOINT5'
#~ rLegOffset = [0,-0.105,0,]
#~ rLegNormal = [0,1,0]
#~ rLegx = 0.09; rLegy = 0.05
#~ fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, 0.01)
#~ 
#~ lLegId = 'lLeg'
#~ lLeg = 'LLEG_JOINT0'
#~ lfoot = 'LLEG_JOINT5'
#~ lLegOffset = [0,-0.105,0]
#~ lLegNormal = [0,1,0]
#~ lLegx = 0.09; lLegy = 0.05
#~ fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, 0.01)
#~ 
#~ rarmId = 'rArm'
#~ rarm = 'RARM_JOINT0'
#~ rHand = 'RARM_JOINT5'
#~ rArmOffset = [-0.05,-0.050,-0.050]
#~ rArmNormal = [1,0,0]
#~ rArmx = 0.024; rArmy = 0.024
#~ fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, 0.01)
#~ 
#~ larmId = 'lArm'
#~ larm = 'LARM_JOINT0'
#~ lHand = 'LARM_JOINT5'
#~ lArmOffset = [-0.05,-0.050,-0.050]
#~ lArmNormal = [1,0,0]
#~ lArmx = 0.024; lArmy = 0.024
#~ fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, 0.01)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()
zeroConf = [0,0,0, 1, 0, 0, 0]
q_0[0:7] = zeroConf
fullBody.setCurrentConfig (q_0)

effectors = [rfoot, lfoot, lHand, rHand]
limbIds = [rLegId, lLegId, larmId, rarmId ]

import numpy as np
#~ effectorName = rfoot
#~ limbId = rLegId
#~ q = fullBody.getSample(limbId, 1)
#~ fullBody.setCurrentConfig(q) #setConfiguration matching sample
#~ qEffector = fullBody.getJointPosition(effectorName)
#~ q0 = quat.Quaternion(qEffector[3:7])
#~ rot = q0.toRotationMatrix() #compute rotation matrix world -> local
#~ p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
#~ rm=np.zeros((4,4))
#~ for i in range(0,3):
	#~ for j in range(0,3):
		#~ rm[i,j] = rot[i,j]
#~ for i in range(0,3):
	#~ rm[i,3] = qEffector[i]
#~ rm[3,3] = 1
#~ invrm = np.linalg.inv(rm)
#~ p = invrm.dot([0,0,0,1])

points = [[],[],[],[]]

success = 0
fails = 0

def printComPosition(nbConfigs):
	for i in range(0,nbConfigs):
		q = fullBody.shootRandomConfig()
		q[0:7] = zeroConf
		fullBody.setCurrentConfig(q) #setConfiguration matching sample
		com = fullBody.getCenterOfMass()
		for x in range(0,3):
			q[x] = -com[x]
		fullBody.setCurrentConfig(q)
		#~ print ("final com" + str(com))
		#~ print ("final com" + str(fullBody.getCenterOfMass()))
		if(fullBody.isConfigValid(q)[0]):
			global success
			success +=1
			for j in range(0,len(effectors)):
				effectorName = effectors[j]
				limbId = limbIds[j]
				qEffector = fullBody.getJointPosition(effectorName)
				q0 = quat.Quaternion(qEffector[3:7])
				rot = q0.toRotationMatrix() #compute rotation matrix world -> local
				p = qEffector[0:3] #(0,0,0) coordinate expressed in effector fram
				rm=np.zeros((4,4))
				for k in range(0,3):
					for l in range(0,3):
						rm[k,l] = rot[k,l]
				for m in range(0,3):
					rm[m,3] = qEffector[m]
				rm[3,3] = 1
				invrm = np.linalg.inv(rm)
				p = invrm.dot([0,0,0,1])
				points[j].append(p)
				#~ print (points[j])
		else:			
			global fails
			fails +=1
			#~ print fullBody.isConfigValid(q)[1]
	for j in range(0,len(limbIds)):
		f1=open('./'+str(limbIds[j])+'_com.erom', 'w+')
		for p in points[j]:
			f1.write(str(p[0]) + "," + str(p[1]) + "," + str(p[2]) + "\n")
		f1.close()

#~ printRootPosition(rLegId, rfoot, nbSamples)
#~ printRootPosition(lLegId, lfoot, nbSamples)
#~ printRootPosition(rarmId, rHand, nbSamples)
#~ printRootPosition(larmId, lHand, nbSamples) 
printComPosition(100000)
print "successes ", success
print "fails  ", fails
