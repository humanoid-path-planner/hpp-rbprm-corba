#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
import time
from hpp.corbaserver.rbprm.tools.display_tools import *
from constraint_to_dae import *

from hpp.gepetto import Viewer


#calling script darpa_hyq_path to compute root path
from . import sideWall_hyq_pathKino as tp


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
r = tp.Viewer (ps,viewerClient=tp.r.client)

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
pp.dt = 0.001

r(q_init)
# computing the contact sequence

tStart = time.time()
#~ configs = fullBody.interpolate(0.08,pathId=1,robustnessTreshold = 2, filterStates = True)
configs = fullBody.interpolate(0.001,pathId=0,robustnessTreshold = 1, filterStates = True)
r(configs[-1])
tInterpolateConfigs = time.time() - tStart




pid = fullBody.isDynamicallyReachableFromState(17,18,True)
import disp_bezier
pp.dt = 0.0001
disp_bezier.showPath(r,pp,pid)

x = [ 2.47985, -0.25492, 0.962874]

createSphere('s',r)
moveSphere('s',r,x)
displayBezierConstraints(r)

path = "/local/dev_hpp/screenBlender/iros2018/polytopes/hyq/path"
for i in range(1,4):
  r.client.gui.writeNodeFile('path_'+str(int(pid[i]))+'_root',path+str(i-1)+'.obj')

r.client.gui.writeNodeFile('s',path+'_S.stl')





noCOQP = 0

for i in range(len(configs)-2):
  pid = fullBody.isDynamicallyReachableFromState(i,i+1)
  if len(pid)==0:
    noCOQP +=1



f = open("/local/fernbac/bench_iros18/success/log_successSideWall.log","a")
f.write("num states : "+str(len(configs))+"\n")
if noCOQP>0:
  f.write("fail, with infeasibles transitions "+str(noCOQP)+"\n")
else:
  f.write("all transition feasibles\n")
f.close()




"""
print "number of configs =", len(configs)
r(configs[-1])

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from fullBodyPlayer import Player
player = Player(fullBody,pp,tp,configs,draw=True,optim_effector=False,use_velocity=dynamic,pathId = 1)

player.displayContactPlan()
"""




"""
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


fullBody.isDynamicallyReachableFromState(0,1)



q_init[-6:] = [1.5,0,0,0,0,0]
s0 = StateHelper.generateStateInContact(fullBody,q_init,dir_init,acc_init)
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s0.sId,[1,1],0.1)
print pid
pp.displayPath(pid)
r(q_far)


### illustration (quasi static), limb RF a deplacer:
q_last = [1.48833,0,0.851836,1,0,0,0,-0.303342,1.18457,-1.50263,-0.293144,-0.140969,1.35962,-0.360039,1.21798,-1.52204,-0.36999,0.640851,0.41639,0,0,0,0,0,0,]

q_unreachable =[1.49333,0,0.851865,1,0,0,0,-0.303326,1.18888,-1.49364,-0.293129,-0.127206,1.34773,-0.362259,0.193872,-1.44217,-0.369971,0.673584,0.360461,0,0,0,0,0,0,]
q_reachable = [1.49333,0,0.851865,1,0,0,0,-0.303326,1.18888,-1.49364,-0.293129,-0.127206,1.34773,-0.127441,0.503988,-1.35928,-0.369971,0.673584,0.360461,0,0,0,0,0,0,]


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
"""
# infeasible :
r([1.46,0,0.851192,1,0,0,0,-0.32001,1.18232,-1.5504,-0.305161,-0.139056,1.37011,-0.36127,1.20462,-1.55472,-0.361812,0.407251,0.775043,0.2,0,0.000953452,0,0,0.000381381,])

 r([1.48,0,0.851289,1,0,0,0,-0.319954,1.20122,-1.51607,-0.305107,-0.083625,1.32092,-0.32883,0.24414,-1.45645,-0.361748,0.487359,0.65692,0.2,0,0.00099159,0,0,0.000381381,])




import generate_contact_sequence_hyq

id = 6
leg = rhLegId

configs=configs[6:8]
cs = generate_contact_sequence_hyq.generateContactSequence(fullBody,configs,6, 7,r)
"""


