#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper


from hpp.gepetto import Viewer


#calling script darpa_hyq_path to compute root path
import sideWall_hyq_pathKino as tp


from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"



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
fullBody.setJointBounds ("base_joint_xyz", [0.8,5.6, -0.5, 0.5, 0.4, 1.2])

#  Setting a number of sample configurations used
dynamic=True

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)

rootName = 'base_joint_xyz'


#  Setting a number of sample configurations used


rootName = 'base_joint_xyz'

def addLimbDb(limbId, heuristicName, loadValues = True, disableEffectorCollision = False):
	fullBody.addLimbDatabase(str(db_dir+limbId+'.db'), limbId, heuristicName,loadValues, disableEffectorCollision)

rfLegId = 'rfleg'
lhLegId = 'lhleg'
rhLegId = 'rhleg'
lfLegId = 'lfleg'

addLimbDb(rfLegId, "manipulability")
addLimbDb(lhLegId, "manipulability")
addLimbDb(rhLegId, "manipulability")
addLimbDb(lfLegId, "manipulability")


q_0 = fullBody.getCurrentConfig();
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(0,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(0,tp.ps.pathLength(0))[0:7]
dir_init = tp.ps.configAtParam(0,0.01)[7:10]
acc_init = tp.ps.configAtParam(0,0.01)[10:13]
dir_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[7:10]
acc_goal = tp.ps.configAtParam(0,tp.ps.pathLength(0))[10:13]
configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

fullBody.setStaticStability(True)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init) ; r(q_init)
s_init = StateHelper.generateStateInContact(fullBody,q_init,dir_init,acc_init)
q_init = s_init.q()
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
s_goal = StateHelper.generateStateInContact(fullBody,q_goal, dir_goal,acc_goal)
q_goal = s_goal.q()

# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = acc_goal[::]
# specifying the full body configurations as start and goal state of the problem
fullBody.setStartStateId(s_init.sId)
fullBody.setEndStateId(s_goal.sId)


r(q_init)
# computing the contact sequence

#~ configs = fullBody.interpolate(0.08,pathId=1,robustnessTreshold = 2, filterStates = True)
configs = fullBody.interpolate(0.005,pathId=0,robustnessTreshold = 2, filterStates = True)


print("number of configs =", len(configs))
r(configs[-1])

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from fullBodyPlayer import Player
player = Player(fullBody,pp,tp,configs,draw=True,optim_effector=False,use_velocity=dynamic,pathId = 1)

#player.displayContactPlan()


from hpp.corbaserver.rbprm.tools.display_tools import *
from constraint_to_dae import *

q=[1.00015,0,0.85,1,0,0,0,-0.304349,0.161872,-1.39148,-0.292088,-0.169484,1.38697,-0.361248,0.194963,-1.44666,-0.370341,-0.170618,1.43348,0.0299991,0,8.5612e-05,2.99991,0,0.0085612,]
q[-6:] = [0]*6
s0 = State(fullBody,q=q,limbsIncontact = [rLegId,lLegId,rarmId,larmId])
s1 = fullBody.createState(q,[rLegId,lLegId,larmId])
pid = fullBody.isReachableFromState(s0,s1)
pid

r(q)
displayOneStepConstraints(r,False)

#~ r(configs[15])
#~ player.interpolate(10,100)

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






