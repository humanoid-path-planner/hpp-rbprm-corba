from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
robot = fullBody.client.basic.robot
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-20,20, -20, 20, -20, 20])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 1


ps = ProblemSolver( fullBody )


rLegId = 'RFoot'
lLegId = 'LFoot'
rarmId = 'RHand'
larmId = 'LHand'
rfoot = 'SpidermanRFootSphere'
lfoot = 'SpidermanLFootSphere'
lHand = 'SpidermanLHandSphere'
rHand = 'SpidermanRHandSphere'
nbSamples = 50000; x = 0.03; y = 0.08
fullBody.addLimb(rLegId,'RThigh_rx','SpidermanRFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(lLegId,'LThigh_rx','SpidermanLFootSphere',[0,0,0],[0,0,1], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(rarmId,'RHumerus_rx','SpidermanRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")
fullBody.addLimb(larmId,'LHumerus_rx','SpidermanLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "EFORT_Normal", 0.01,"_6_DOF")

fullBody.runLimbSampleAnalysis(rLegId, "jointLimitsDistance", True)
fullBody.runLimbSampleAnalysis(lLegId, "jointLimitsDistance", True)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()
zeroConf = [0,0,0, 1, 0, 0, 0]
q_0[0:7] = zeroConf
fullBody.setCurrentConfig (q_0)

effectors = [rfoot, lfoot, lHand, rHand]
limbIds = [rLegId, lLegId, larmId, rarmId ]

import numpy as np

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
