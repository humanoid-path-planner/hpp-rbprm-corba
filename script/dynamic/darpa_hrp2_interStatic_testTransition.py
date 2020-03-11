from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from tools import *
import darpa_hrp2_path as tp
import time
import omniORB.any
import numpy as np
from numpy import linalg as LA
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper

packageName = "hrp2_14_description"
meshPackageName = "hrp2_14_description"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "hrp2_14"
urdfSuffix = "_reduced"
srdfSuffix = ""
pId = tp.ps.numberPaths() -1
fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz",  [-1.2,1.5,-0.05 ,0.05, 0.55, 0.85])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-0,0,-0,0,-0,0,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",omniORB.any.to_any(tp.aMax))
ps.client.problem.setParameter("vMax",omniORB.any.to_any(tp.vMax))

r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)

q_init =[0, 0, 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
fullBody.setReferenceConfig (q_ref)



#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
lLegId = 'hrp2_lleg_rom'
tStart = time.time()


rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.105]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 50000, "forward", 0.1,"_6_DOF")
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 100000, "fixedStep08", 0.01,"_6_DOF",limbOffset=rLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/empty_com_constraints.obj")
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.105]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 50000, "forward", 0.1,"_6_DOF")
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep08", 0.01,"_6_DOF",limbOffset=lLegLimbOffset,kinematicConstraintsPath = "package://hpp-rbprm-corba/com_inequalities/empty_com_constraints.obj")
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")

## Add arms (not used for contact) : 


tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")


"""
fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
tLoad =  time.time() - tStart
print "Load databases in : "+str(tLoad)+" s"
"""


q_0 = fullBody.getCurrentConfig(); 
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)



eps=0.0001
configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(pId,eps)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.0001)[0:7]
dir_init = tp.ps.configAtParam(pId,eps)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-eps)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 1
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]





# Randomly generating a contact configuration at q_init
fullBody.setStaticStability(True)
fullBody.setCurrentConfig (q_init)
r(q_init)
q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('hrp2_14/BODY',0.3)
r(q_init)


fullBody.setStartState(q_init,[rLegId,lLegId])
fullBody.setEndState(q_goal,[rLegId,lLegId])
fullBody.setStaticStability(True) # only set it after the init/goal configuration are computed



from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

import fullBodyPlayerHrp2

tStart = time.time()
configs = fullBody.interpolate(0.01,pathId=pId,robustnessTreshold = robTreshold, filterStates = True, testReachability=True, quasiStatic=True)
tInterpolate = time.time()-tStart
print("number of configs : ", len(configs))
print("generated in "+str(tInterpolate)+" s")
r(configs[len(configs)-2])




f = open("/local/fernbac/bench_iros18/kin_constraint_tog/without_kin_constraints.log","a")

global num_transition
global valid_transition
global length_traj
global valid_length
global dt
num_transition = 0.
valid_transition = 0.
dt = 0.001
valid_length = 0.
length_traj = 0.


def check_traj(s,c0,c1,reverse=False):
  global length_traj
  global valid_length
  global dt  
  dist =  LA.norm(c1-c0)
  if dist < dt :
    return True
  length_traj += dist
  current_dist = 0.
  dtu = dt/dist  
  successProj_total = True
  if not reverse :
    u = 0.
    while u < 1.:
      c = c0*(1.-u) + c1*u
      successProj = s.projectToCOM(c.transpose().tolist(),20) 
      current_dist += dt
      if successProj :
        valid_length += dt
      else :
        print("projection failed.")
        successProj_total = False
      u += dtu
      
    successProj = s.projectToCOM(c1.transpose().tolist(),20) 
    if successProj :
      valid_length += dist - current_dist
    else :
      print("projection failed.")
      successProj_total = False  
  else :
    u = 1.
    while u > 0.:
      c = c0*(1.-u) + c1*u
      successProj = s.projectToCOM(c.transpose().tolist(),20) 
      current_dist += dt
      if successProj :
        valid_length += dt
      else :
        print("projection failed.")
        successProj_total = False
      u -= dtu
      
    successProj = s.projectToCOM(c0.transpose().tolist(),20) 
    if successProj :
      valid_length += dist - current_dist
    else :
      print("projection failed.")
      successProj_total = False 
  
  return successProj_total

def check_kin_feasibility(s0Id,s1Id):
  res = fullBody.isReachableFromState(s0Id,s1Id,True)
  if not res[0]:
    print("Error : isReachable returned False !")
    raise Exception('Error : isReachable returned False !') 
  r(configs[s0Id])
  c0 = np.array(fullBody.getCenterOfMass())
  c1 = np.array(res[1])
  c2 = np.array(res[2])
  r(configs[s1Id])
  c3 = np.array(fullBody.getCenterOfMass())
  s0 = State(fullBody,sId = s0Id)
  s1 = State(fullBody,sId = s1Id)  
  s0_orig = State(s0.fullBody,q=s0.q(),limbsIncontact=s0.getLimbsInContact())
  s1_orig = State(s1.fullBody,q=s1.q(),limbsIncontact=s1.getLimbsInContact())  
  moving_limb = s0.contactsVariations(s1)
  smid,successMid = StateHelper.removeContact(s0_orig,moving_limb[0])
  if not successMid :
    return False
  
  successFeasible = True
  print("check first part")
  successFeasible = successFeasible and check_traj(s0_orig,c0,c1)
  print("check mid part")
  successFeasible = successFeasible and check_traj(smid,c1,c2)
  print("check last part")
  successFeasible = successFeasible and check_traj(s1_orig,c2,c3,True)
  return successFeasible
  

for i in range(len(configs)-2):
  print("check traj between state "+str(i)+" and "+str(i+1))
  success = check_kin_feasibility(i,i+1)
  num_transition +=1.
  if success :
    valid_transition +=1.
  


f.write("num_transition           "+str(num_transition)+"\n")
f.write("valid_transition         "+str(valid_transition)+"\n")
if num_transition > 0:
  f.write("valid_transition_percent "+str(float(valid_transition/num_transition)*100.)+" %\n")
f.write("traj_length              "+str(length_traj)+"\n")
f.write("valid_length             "+str(valid_length)+"\n")
if length_traj > 0 :
  f.write("valid_traj_percent       "+str(float(valid_length/length_traj)*100.)+" %\n")

f.close()



#wid = r.client.gui.getWindowID("window_hpp_")
#r.client.gui.attachCameraToNode( 'hrp2_14/BODY_0',wid)





