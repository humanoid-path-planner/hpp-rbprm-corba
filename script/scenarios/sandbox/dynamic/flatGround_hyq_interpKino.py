#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
from hpp.corbaserver.rbprm.tools.display_tools import *
#calling script darpa_hyq_path to compute root path
from . import flatGround_hyq_pathKino as tp

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
fullBody.setJointBounds ("base_joint_xyz", [-5,5, -1.5, 1.5, 0.5, 0.8])

#  Setting a number of sample configurations used
nbSamples = 20000
dynamic=True

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps,viewerClient=tp.r.client,displayCoM = True)

#  Setting a number of sample configurations used
nbSamples = 20000
rootName = 'base_joint_xyz'
#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_3_DOF"
# string identifying the limb
rfLegId = 'rfleg'
# First joint of the limb, as in urdf file
rfLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfFoot = 'rf_foot_joint'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rfLegId,rfLeg,rfFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)
fullBody.runLimbSampleAnalysis(rfLegId, "jointLimitsDistance", True)

lhLegId = 'lhleg'
lhLeg = 'lh_haa_joint'
lhFoot = 'lh_foot_joint'
fullBody.addLimb(lhLegId,lhLeg,lhFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)
fullBody.runLimbSampleAnalysis(lhLegId, "jointLimitsDistance", True)

lfLegId = 'lfleg'
lfLeg = 'lf_haa_joint'
lfFoot = 'lf_foot_joint'
fullBody.addLimb(lfLegId,lfLeg,lfFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)
fullBody.runLimbSampleAnalysis(lfLegId, "jointLimitsDistance", True)

rhLegId = 'rhleg'
rhLeg = 'rh_haa_joint'
rhFoot = 'rh_foot_joint'
fullBody.addLimb(rhLegId,rhLeg,rhFoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)
fullBody.runLimbSampleAnalysis(rhLegId, "jointLimitsDistance", True)


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

q_far = q_init[::]
q_far[2] = -5

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)


configs = fullBody.interpolate(0.001,pathId=0,robustnessTreshold = 2, filterStates = True)
r(configs[-1])

#~ r.loadObstacleModel ('hpp-rbprm-corba', "darpa", "contact")
"""
from fullBodyPlayer import Player
player = Player(fullBody,pp,tp,configs,draw=True,optim_effector=False,use_velocity=dynamic,pathId = 0)

player.displayContactPlan()

#player.interpolate(5,len(configs)-2)

#player.play()

"""

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






