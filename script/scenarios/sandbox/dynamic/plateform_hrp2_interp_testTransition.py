from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
import time
from constraint_to_dae import *
from hpp.corbaserver.rbprm.rbprmstate import State,StateHelper
from hpp.corbaserver.rbprm.tools.display_tools import *
from . import plateform_hrp2_path as tp
import time

tPlanning = tp.tPlanning


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
fullBody.setJointBounds ("base_joint_xyz", [-1,2, -0.5, 0.5, 0.5, 0.8])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(tp.extraDof)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([-1,1,-1,1,-0.5,0.5,0,0,0,0,0,0])
ps = tp.ProblemSolver( fullBody )
ps.client.problem.setParameter("aMax",tp.aMax)
ps.client.problem.setParameter("vMax",tp.vMax)

r = tp.Viewer (ps,viewerClient=tp.r.client,displayArrows = True, displayCoM = True)


q_init =[0., 0., 0.648702, 1.0, 0.0 , 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.17,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,0,0,0,0,0,0]; r (q_init)
q_ref = q_init[::]
fullBody.setCurrentConfig (q_init)
qfar=q_ref[::]
qfar[2] = -5

#~ AFTER loading obstacles
rLegId = 'hrp2_rleg_rom'
lLegId = 'hrp2_lleg_rom'
tStart = time.time()


rLeg = 'RLEG_JOINT0'
rLegOffset = [0,0,-0.0955]
rLegLimbOffset=[0,0,-0.035]#0.035
rLegNormal = [0,0,1]
rLegx = 0.09; rLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_rleg_db.db",rLegId,"forward")
fullBody.addLimb(rLegId,rLeg,'',rLegOffset,rLegNormal, rLegx, rLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=rLegLimbOffset)
fullBody.runLimbSampleAnalysis(rLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(rLegId, "./db/hrp2_rleg_db.db")

lLeg = 'LLEG_JOINT0'
lLegOffset = [0,0,-0.0955]
lLegLimbOffset=[0,0,0.035]
lLegNormal = [0,0,1]
lLegx = 0.09; lLegy = 0.05
#fullBody.addLimbDatabase("./db/hrp2_lleg_db.db",lLegId,"forward")
fullBody.addLimb(lLegId,lLeg,'',lLegOffset,rLegNormal, lLegx, lLegy, 100000, "fixedStep1", 0.01,"_6_DOF",limbOffset=lLegLimbOffset)
fullBody.runLimbSampleAnalysis(lLegId, "ReferenceConfiguration", True)
#fullBody.saveLimbDatabase(lLegId, "./db/hrp2_lleg_db.db")
fullBody.setReferenceConfig (q_ref)
## Add arms (not used for contact) :


tGenerate =  time.time() - tStart
print("generate databases in : "+str(tGenerate)+" s")


q_0 = fullBody.getCurrentConfig();
#~ fullBody.createOctreeBoxes(r.client.gui, 1, rarmId, q_0,)




configSize = fullBody.getConfigSize() -fullBody.client.basic.robot.getDimensionExtraConfigSpace()

q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.ps.configAtParam(pId,0.01)[0:7] # use this to get the correct orientation
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.ps.configAtParam(pId,tp.ps.pathLength(pId))[0:7]
dir_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS:tp.indexECS+3]
acc_init = tp.ps.configAtParam(pId,0.01)[tp.indexECS+3:tp.indexECS+6]
dir_goal = tp.ps.configAtParam(pId,tp.ps.pathLength(pId)-0.01)[tp.indexECS:tp.indexECS+3]
acc_goal = [0,0,0]

robTreshold = 3
# copy extraconfig for start and init configurations
q_init[configSize:configSize+3] = dir_init[::]
q_init[configSize+3:configSize+6] = acc_init[::]
q_goal[configSize:configSize+3] = dir_goal[::]
q_goal[configSize+3:configSize+6] = [0,0,0]


# FIXME : test
q_init[2] = q_ref[2]+0.011
q_goal[2] = q_ref[2]+0.011

fullBody.setStaticStability(True)
# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
r(q_init)
#q_init = fullBody.generateContacts(q_init,dir_init,acc_init,robTreshold)
r(q_init)

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
#q_goal = fullBody.generateContacts(q_goal, dir_goal,acc_goal,robTreshold)
r(q_goal)

# specifying the full body configurations as start and goal state of the problem
r.addLandmark('hrp2_14/BODY',0.3)
r(q_init)


fullBody.setStartState(q_init,[lLegId,rLegId])
fullBody.setEndState(q_goal,[lLegId,rLegId])


#p = fullBody.computeContactPointsAtState(init_sid)
#p = fullBody.computeContactPointsAtState(int_sid)


"""
q = q_init[::]
q[0] += 0.3
q = fullBody.generateContacts(q,dir_init,acc_init,robTreshold)
mid_sid = fullBody.addState(q,[lLegId,rLegId])
"""

from hpp.gepetto import PathPlayer
pp = PathPlayer (fullBody.client.basic, r)

from . import fullBodyPlayerHrp2

"""
tStart = time.time()
configsFull = fullBody.interpolate(0.001,pathId=pId,robustnessTreshold = 1, filterStates = False)
tInterpolateConfigs = time.time() - tStart
print "number of configs :", len(configsFull)
"""

q_init[0] += 0.05
createSphere('s',r)

n = [0,0,1]
p = [0,0.1,0]

q_init[-6:-3] = [0.2,0,0]
q_goal[-6:-3] = [0.1,0,0]
sf = State(fullBody,q=q_goal,limbsIncontact=[lLegId,rLegId])
si = State(fullBody,q=q_init,limbsIncontact=[lLegId,rLegId])


n = [0.0, -0.42261828000211843, 0.9063077785212101]
p = [0.775, 0.22, -0.019]
moveSphere('s',r,p)
smid,success = StateHelper.addNewContact(si,lLegId,p,n,100)
assert(success)
smid2,success = StateHelper.addNewContact(sf,lLegId,p,n,100)
assert(success)
r(smid.q())

sf2 = State(fullBody,q=q_goal,limbsIncontact=[lLegId,rLegId])


"""
fullBody.isDynamicallyReachableFromState(smid.sId,smid2.sId,True)
fullBody.isDynamicallyReachableFromState(smid.sId,smid2.sId,timings=[0.4,0.2,0.4])
fullBody.isDynamicallyReachableFromState(si.sId,smid.sId,timings=[0.8,0.6,0.8])
fullBody.isDynamicallyReachableFromState(smid2.sId,sf.sId,timings=[0.8,0.6,0.8])

import disp_bezier
pp.dt = 0.00001
disp_bezier.showPath(r,pp,pid)

x = [0.776624, 0.219798, 0.846351]


moveSphere('s',r,x)
displayBezierConstraints(r)

path = "/local/dev_hpp/screenBlender/iros2018/polytopes/platform/path"
for i in range(1,4):
  r.client.gui.writeNodeFile('path_'+str(int(pid[i]))+'_root',path+str(i-1)+'.obj')

r.client.gui.writeNodeFile('s',path+'_S.stl')

"""

"""
com = fullBody.getCenterOfMass()
com[1] = 0
"""
"""
pids = []
pids += [fullBody.isDynamicallyReachableFromState(si.sId,smid.sId)]
pids += [fullBody.isDynamicallyReachableFromState(smid.sId,smid2.sId)]
pids += [fullBody.isDynamicallyReachableFromState(smid2.sId,sf2.sId)]

for pid in pids :
  if pid > 0:
    print "success"
    #pp.displayPath(pid,color=r.color.blue)
    #r.client.gui.setVisibility('path_'+str(pid)+'_root','ALWAYS_ON_TOP')
  else:
    print "fail."
"""
"""

n = [0,0,1]
p = [1.15,0.1,0]
moveSphere('s',r,p)

sE,success = StateHelper.addNewContact(si,lLegId,p,n)
assert(success)
p = [1.15,-0.1,0]
sfe, success = StateHelper.addNewContact(sE,rLegId,p,n)
assert(success)

pids = []
pids += [fullBody.isDynamicallyReachableFromState(si.sId,sE.sId)]
pids += [fullBody.isDynamicallyReachableFromState(sE.sId,sfe.sId)]
for pid in pids :
  if pid > 0:
    print "success"
    pp.displayPath(pid,color=r.color.blue)
    r.client.gui.setVisibility('path_'+str(pid)+'_root','ALWAYS_ON_TOP')
  else:
    print "fail."

"""

configs = []
configs += [si.q()]
configs += [smid.q()]
configs += [smid2.q()]
configs += [sf2.q()]



from planning.config import *
from generate_contact_sequence import *

beginState = si.sId
endState = sf2.sId
cs = generateContactSequence(fullBody,configs,beginState, endState,r)



filename = OUTPUT_DIR + "/" + OUTPUT_SEQUENCE_FILE
cs.saveAsXML(filename, "ContactSequence")
print("save contact sequence : ",filename)






