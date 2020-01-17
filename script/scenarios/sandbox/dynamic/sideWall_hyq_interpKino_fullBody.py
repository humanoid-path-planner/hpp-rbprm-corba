#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper

from hpp.corbaserver.rbprm.display_tools import *
from constraint_to_dae import *

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
fullBody.setJointBounds("base_joint_xyz", [0.8,5.6, -0.5, 0.5, 0.4, 1.2])
#  Setting a number of sample configurations used
dynamic=True

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)

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
pp.dt = 0.0001

r(q_init)
# computing the contact sequence

#~ configs = fullBody.interpolate(0.08,pathId=1,robustnessTreshold = 2, filterStates = True)
configs = fullBody.interpolate(0.001,pathId=0,robustnessTreshold = 1, filterStates = True)
r(configs[-1])




print("number of configs =", len(configs))
r(configs[-1])

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from fullBodyPlayer import Player
player = Player(fullBody,pp,tp,configs,draw=True,optim_effector=False,use_velocity=False,pathId = 0)

#player.displayContactPlan()

player.interpolate(0,len(configs)-1)

#### whole body :
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *
from disp_bezier import *

beginState = 0
endState = len(configs)-2
time_per_paths = []
total_paths_ids = []
merged_paths_ids = []

for id_state in range(beginState,endState):

  pid = fullBody.isDynamicallyReachableFromState(id_state,id_state+1,True)
  assert(len(pid) > 0 and "isDynamicallyReachable failed for state "+str(id_state))
  showPath(r,pp,pid)

  pId1 = int(pid[0]) ; pId2 = int(pid[1]) ; pId3=int(pid[2])
  paths_ids = fullBody.effectorRRTFromPosBetweenState(id_state,id_state+1,pId1,pId2,pId3,1)

  time_per_paths += [ps.pathLength(pId1)]
  time_per_paths += [ps.pathLength(pId2)]
  time_per_paths += [ps.pathLength(pId3)]
  total_paths_ids += paths_ids[:-1]
  merged_paths_ids += [paths_ids[-1]]



end_effector_joints_names=["lf_foot_joint","lh_foot_joint","rf_foot_joint","rh_foot_joint"]
colors=[r.color.red,r.color.lightRed,r.color.brow,r.color.lightBrown]
#fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof+1) # needed because the intermediate path have one more degree of freedom
displayFeetTrajectory(pp,end_effector_joints_names,colors,merged_paths_ids)
#fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)


trajectory = gen_trajectory_to_play(fullBody,pp,total_paths_ids,time_per_paths)
play_trajectory(fullBody,pp, trajectory)










