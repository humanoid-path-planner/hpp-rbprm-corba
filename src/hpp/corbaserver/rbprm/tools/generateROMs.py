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
fullBody.setJointBounds ("base_joint_xyz", [-1,1, -4, -1, 1, 2.2])

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 50000

ps = ProblemSolver( fullBody )

#~ AFTER loading obstacles
rLegId = '7rLeg'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, 0.01)

lLegId = '8lLeg'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, nbSamples, 0.01)

rarmId = '3Rarm'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [-0.05,-0.050,-0.050]
rArmNormal = [1,0,0]
rArmx = 0.024; rArmy = 0.024
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, 0.01)


#~ AFTER loading obstacles
larmId = '4Larm'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, 0.01)



q_0 = fullBody.getCurrentConfig ()

limit = nbSamples-1;
f1=open('./data/roms/rleg.erom', 'w+')
for i in range(0,limit):
	q = fullBody.getSamplePosition(rLegId,i)
	f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
f1.close()


f1=open('./data/roms/lleg.erom', 'w+')
for i in range(0,limit):
	q = fullBody.getSamplePosition(lLegId,i)
	f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
f1.close()
	
	
f1=open('./data/roms/rarm.erom', 'w+')
for i in range(0,limit):
	q = fullBody.getSamplePosition(rarmId,i)
	f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
f1.close()
	
	
f1=open('./data/roms/larm.erom', 'w+')
for i in range(0,limit):
	q = fullBody.getSamplePosition(larmId,i)
	f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
f1.close()


