#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import slalom_hyq_pathKino as tp

from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"

pathId = tp.ps.numberPaths()-1

packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.setJointBounds ("base_joint_xyz", [-6,6, -2.5, 2.5, 0.0, 1.])

#  Setting a number of sample configurations used
nbSamples = 20000
dynamic=True

ps = tp.ProblemSolver(fullBody)
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)
r = tp.Viewer (ps,viewerClient=tp.r.client)

rootName = 'base_joint_xyz'


def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

rLegId = 'rfleg'
lLegId = 'lhleg'
rarmId = 'rhleg'
larmId = 'lfleg'

addLimbDb(rLegId, "manipulability")
addLimbDb(lLegId, "manipulability")
addLimbDb(rarmId, "manipulability")
addLimbDb(larmId, "manipulability")

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(0,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pathId,tp.ps.pathLength(pathId))[0:7]
dir_init = tp.ps.configAtParam(pathId,0.01)[7:10]
acc_init = tp.ps.configAtParam(pathId,0.01)[10:13]
dir_goal = tp.ps.configAtParam(pathId,tp.ps.pathLength(pathId))[7:10]
acc_goal = tp.ps.configAtParam(pathId,tp.ps.pathLength(pathId))[10:13]
configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]


fullBody.setStaticStability(False)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init,2)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,2)


# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[larmId,rLegId,rarmId,lLegId])
fullBody.setEndState(q_goal,[larmId,rLegId,rarmId,lLegId])


r(q_init)
# computing the contact sequence

configs = fullBody.interpolate(0.001,pathId=pathId,robustnessTreshold = 0, filterStates = True)


print "number of configs =", len(configs)
r(configs[-1])


"""
from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayer
player = fullBodyPlayer.Player(fullBody,pp,tp,configs,draw=True,optim_effector=True,use_velocity=dynamic,pathId = pathId)

#player.displayContactPlan()

r(configs[2])
player.interpolate(2,40)
"""
#player.play()



"""

camera = [0.5681925415992737,
 -6.707448482513428,
 2.5206544399261475,
 0.8217507600784302,
 0.5693002343177795,
 0.020600343123078346,
 0.01408931240439415]
r.client.gui.setCameraTransform(0,camera)

"""




"""
import hpp.corbaserver.rbprm.tools.cwc_trajectory
import hpp.corbaserver.rbprm.tools.path_to_trajectory
import hpp.corbaserver.rbprm.tools.cwc_trajectory_helper

reload(hpp.corbaserver.rbprm.tools.cwc_trajectory)
reload(hpp.corbaserver.rbprm.tools.path_to_trajectory)
reload(hpp.corbaserver.rbprm.tools.cwc_trajectory_helper)
reload(fullBodyPlayer)


"""


