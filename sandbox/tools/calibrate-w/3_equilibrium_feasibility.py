from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
import sys

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
#~ fullBody.setJointBounds ("base_joint_xyz", [-1,2, -2, 1, 0.5, 2.5])

#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
rLeg = 'RLEG_JOINT0'
rLegOffset = [0,-0.105,0,]
rLegNormal = [0,1,0]
rLegx = 0.09; rLegy = 0.05
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 10000, "manipulability", 0.03)

lLegId = 'hrp2_lleg_rom'
lLeg = 'LLEG_JOINT0'
lLegOffset = [0,-0.105,0]
lLegNormal = [0,1,0]
lLegx = 0.09; lLegy = 0.05
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 10000, "manipulability", 0.03)

rarmId = 'hrp2_rarm_rom'
rarm = 'RARM_JOINT0'
rHand = 'RARM_JOINT5'
rArmOffset = [-0.05,-0.050,-0.050]
rArmNormal = [1,0,0]
rArmx = 0.024; rArmy = 0.024
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, 10000, "EFORT", 0.05)

larmId = 'hrp2_larm_rom'
larm = 'LARM_JOINT0'
lHand = 'LARM_JOINT5'
lArmOffset = [-0.05,-0.050,-0.050]
lArmNormal = [1,0,0]
lArmx = 0.024; lArmy = 0.024
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, 10000, "EFORT", 0.05)

scale = sys.argv[len(sys.argv)-2]
scene = sys.argv[len(sys.argv)-1]
#~ configFile = sys.argv[len(sys.argv)-1]

import pickle
sFile = "false_negative_configs_"+scene+'_'+scale+'.pkl'
pkl_file = open(sFile, 'rb')
falseNeg = pickle.load(pkl_file)
pkl_file.close()

ps = ProblemSolver( fullBody )
r = Viewer (ps)
r.loadObstacleModel ('hpp-rbprm-corba', scene, "planning")

q_init =  [
        0.1, -0.82, 0.648702, 1.0, 0.0 , 0.0, 0.0,                         	 # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17, # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ]; r (q_init)

fullBody.setCurrentConfig (q_init)

confsize = len(falseNeg[0])

falseFalseNegative = 0
trueFalseNegative = 0

nbnegative = 0
totalconfigs = 0



q_init = fullBody.getCurrentConfig();
for q in falseNeg:
	q_init[0:confsize] = q[0:confsize]
	totalconfigs = totalconfigs + 1
	#~ print "avant " + str(fullBody.isConfigValid(q_init)) + str(q_init)
	#~ q_init = fullBody.makeCollisionFree(q_init)
	#~ print "apres " + str(fullBody.isConfigValid(q_init))  + str(q_init)
	#~ raise ValueError ("tg")
	#~ fullBody.canGenerateLimbContact(limb, q_init)
	if (fullBody.canGenerateBalancedContact(q_init, [0,0,1])):
		trueFalseNegative = trueFalseNegative + 1
	else:
		falseFalseNegative = falseFalseNegative +1
			
				#~ 

f = open('log.txt', 'a')
f.write("size 1 \n \t true false negative " + str(float(trueFalseNegative)/float(len(falseNeg))) + "\n")
f.write("size 1 \n \t false false negative " + str(float(falseFalseNegative)/float(len(falseNeg))) + "\n")
f.close()

#~ for limb in positive.keys():
	#~ print 'positive ' + limb + ': ' + str(len(positive[limb]))
	#~ 
#~ for limb in negative.keys():
	#~ print 'negative ' + limb + ': ' + str(len(negative[limb]))
