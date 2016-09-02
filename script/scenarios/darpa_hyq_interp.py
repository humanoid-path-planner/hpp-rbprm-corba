#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer

#calling script darpa_hyq_path to compute root path
import darpa_hyq_path as tp

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
fullBody.setJointBounds ("base_joint_xyz", [-2,5, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 20000

ps = tp.ProblemSolver(fullBody)
r = tp.Viewer (ps)

rootName = 'base_joint_xyz'

#  Creating limbs
# cType is "_3_DOF": positional constraint, but no rotation (contacts are punctual)
cType = "_3_DOF"
# string identifying the limb
rLegId = 'rfleg'
# First joint of the limb, as in urdf file
rLeg = 'rf_haa_joint'
# Last joint of the limb, as in urdf file
rfoot = 'rf_foot_joint'
# Specifying the distance between last joint and contact surface
offset = [0.,-0.021,0.]
# Specifying the contact surface direction when the limb is in rest pose
normal = [0,1,0]
# Specifying the rectangular contact surface length
legx = 0.02; legy = 0.02
# remaining parameters are the chosen heuristic (here, manipulability), and the resolution of the octree (here, 10 cm).
fullBody.addLimb(rLegId,rLeg,rfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.1, cType)

lLegId = 'lhleg'
lLeg = 'lh_haa_joint'
lfoot = 'lh_foot_joint'
fullBody.addLimb(lLegId,lLeg,lfoot,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

rarmId = 'rhleg'
rarm = 'rh_haa_joint'
rHand = 'rh_foot_joint'
fullBody.addLimb(rarmId,rarm,rHand,offset,normal, legx, legy, nbSamples, "manipulability", 0.05, cType)

larmId = 'lfleg'
larm = 'lf_haa_joint'
lHand = 'lf_foot_joint'
fullBody.addLimb(larmId,larm,lHand,offset,normal, legx, legy, nbSamples, "forward", 0.05, cType)

q_0 = fullBody.getCurrentConfig(); 
q_init = fullBody.getCurrentConfig(); q_init[0:7] = tp.q_init[0:7]
q_goal = fullBody.getCurrentConfig(); q_goal[0:7] = tp.q_goal[0:7]

# Randomly generating a contact configuration at q_init
fullBody.setCurrentConfig (q_init)
q_init = fullBody.generateContacts(q_init, [0,0,1])

# Randomly generating a contact configuration at q_end
fullBody.setCurrentConfig (q_goal)
q_goal = fullBody.generateContacts(q_goal, [0,0,1])

# specifying the full body configurations as start and goal state of the problem
fullBody.setStartState(q_init,[])
fullBody.setEndState(q_goal,[rLegId,lLegId,rarmId,larmId])


r(q_init)
# computing the contact sequence
configs = fullBody.interpolate(0.1, 1, 5)

#~ r.loadObstacleModel ('hpp-rbprm-corba', "darpa", "contact")

# calling draw with increasing i will display the sequence
i = 0;
fullBody.draw(configs[i],r); i=i+1; i-1

from hpp.gepetto import PathPlayer
pp = PathPlayer (ps.robot.client.basic, r)


from hpp.corbaserver.rbprm.tools.cwc_trajectory import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *

def displayComPath(pathId,color=[0.,0.75,0.15,0.9]) :
	pathPos=[]
	length = pp.end*pp.client.problem.pathLength (pathId)
	t = pp.start*pp.client.problem.pathLength (pathId)
	while t < length :
		q = pp.client.problem.configAtParam (pathId, t)
		pp.publisher.robot.setCurrentConfig(q)
		q = pp.publisher.robot.getCenterOfMass()
		pathPos = pathPos + [q[:3]]
		t += pp.dt
	nameCurve = "path_"+str(pathId)+"_com"
	pp.publisher.client.gui.addCurve(nameCurve,pathPos,color)
	pp.publisher.client.gui.addToGroup(nameCurve,pp.publisher.sceneName)
	pp.publisher.client.gui.refresh()

res = []
trajec = []
from hpp import Error as hpperr
import sys
numerror = 0
def act(i, optim):
	global numerror
	try:
		pid, trajectory = solve_com_RRT(fullBody, configs, i, True, 0.6, 0.2, False, optim, False, True)
		displayComPath(pid)
		#~ pp(pid)
		global res
		res = res + [pid]
		global trajec
		trajec = trajec + gen_trajectory_to_play(fullBody, pp, trajectory, 1)
	except hpperr as e:
		print "failed at id " + str(i) , e.strerror
		numerror+=1
	except ValueError as e:
		print "failed at id " + str(i) , e
		numerror+=1
	except IndexError as e:
		print "failed at id " + str(i) , e
		numerror+=1
	except Exception as e:
		print e
		numerror+=1
	except:
		return
	
def displayInSave(pp, pathId, configs):
	length = pp.end*pp.client.problem.pathLength (pathId)
	t = pp.start*pp.client.problem.pathLength (pathId)
	while t < length :
		q = pp.client.problem.configAtParam (pathId, t)
		configs.append(q)
		t += (pp.dt * pp.speed)

	
#~ fullBody.exportAll(r, respath, 'darpa_hyq_full');
	
for i in range(11,20):
	act(i, 60)
	#~ if i % 10 == 0:			
		#~ respath = []
		#~ for p in res:
			#~ print p
			#~ displayInSave(pp,p, respath)
		#~ fullBody.exportAll(r, respath, 'darpa_hyq_full');



pp.setSpeed(2)

for p in res:
	pp(p)
		

import time

respath = []
for p in res:
	print p
	#~ displayInSave(pp,p, respath)
#~ fullBody.exportAll(r, respath, 'darpa_hyq_full'+str(time()))



#~ for i in range(5,35):
	#~ act(i, 50)


	

#~ from hpp.gepetto.blender.exportmotion import exportPath
#~ for p in res:
	#~ exportPath(r,fullBody.client.basic.robot,fullBody.client.basic.problem,p,0.1,'test'+str(p)+'.txt')

print "tg"
