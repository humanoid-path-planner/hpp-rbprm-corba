from cwc import cone_optimization
from obj_to_constraints import ineq_from_file, rotate_inequalities
import numpy as np
import math

import hpp.corbaserver.rbprm.data.com_inequalities as ine
ineqPath = ine.__path__[0] +"/"

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()
	robot.setCurrentConfig(save)
	return com

constraintsComLoaded = {}

def __get_com_constraint(fullBody, state, config, limbsCOMConstraints, interm = False):
	global constraintsLoaded
	As = [];	bs = []
	fullBody.setCurrentConfig(config)
	contacts = []
	for i, v in limbsCOMConstraints.iteritems():
		if not constraintsComLoaded.has_key(i):
			constraintsComLoaded[i] = ineq_from_file(ineqPath+v['file'])
		print "inter", interm
		print "intermed", fullBody.isLimbInContactIntermediary(i, state)
		print "inter", fullBody.isLimbInContact(i, state)
		contact = (interm and fullBody.isLimbInContactIntermediary(i, state)) or (not interm and fullBody.isLimbInContact(i, state))
		if contact:
			ineq = constraintsComLoaded[i]
			qEffector = fullBody.getJointPosition(v['effector'])
			tr = quaternion_matrix(qEffector[3:7])			
			tr[:3,3] = np.array(qEffector[0:3])
			ineq_r = rotate_inequalities(ineq, tr)
			As.append(ineq_r.A); bs.append(ineq_r.b);
			print 'contact', v['effector']
			contacts.append(v['effector'])
	print 'contacts', contacts
	return [np.vstack(As), np.hstack(bs)]
		

def gen_trajectory(fullBody, states, state_id, computeCones = False, mu = 1, dt=0.2, reduce_ineq = True, verbose = False, limbsCOMConstraints = None):
	init_com = __get_com(fullBody, states[state_id])
	end_com = __get_com(fullBody, states[state_id+1])
	p, N = fullBody.computeContactPoints(state_id)
	mass = fullBody.getMass()
	t_end_phases = [0]
	[t_end_phases.append(t_end_phases[-1]+1) for _ in range(len(p))]
	cones = None
	if(computeCones):
		cones = [fullBody.getContactCone(state_id, mu)[0]]
		if(len(p) > 2):
			cones.append(fullBody.getContactIntermediateCone(state_id, mu)[0])
		if(len(p) > len(cones)):
			cones.append(fullBody.getContactCone(state_id+1, mu)[0])
	print "num cones ", len(cones)
	
	COMConstraints = None
	if(not (limbsCOMConstraints == None)):
		print "retrieving COM constraints"
		COMConstraints = [__get_com_constraint(fullBody, state_id, states[state_id], limbsCOMConstraints)]
		if(len(p) > 2):
			COMConstraints.append(__get_com_constraint(fullBody, state_id, states[state_id], limbsCOMConstraints, True))
		if(len(p) > len(COMConstraints)):
			COMConstraints.append(__get_com_constraint(fullBody, state_id + 1, states[state_id + 1], limbsCOMConstraints))
		print "num com constraints", len(COMConstraints)
		
	return cone_optimization(p, N, [init_com + [0,0,0], end_com + [0,0,0]], t_end_phases[1:], dt, cones, COMConstraints, mu, mass, 9.81, reduce_ineq, verbose)

def draw_trajectory(fullBody, states, state_id, computeCones = False, mu = 1,  dt=0.2, reduce_ineq = True, verbose = False, limbsCOMConstraints = None):
	var_final, params = gen_trajectory(fullBody, states, state_id, computeCones, mu , dt, reduce_ineq, verbose, limbsCOMConstraints)
	p, N = fullBody.computeContactPoints(state_id)
	from mpl_toolkits.mplot3d import Axes3D
	import matplotlib.pyplot as plt
	
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	n = 100
	points = var_final['x']
	xs = [points[i] for i in range(0,len(points),6)]
	ys = [points[i] for i in range(1,len(points),6)]
	zs = [points[i] for i in range(2,len(points),6)]
	ax.scatter(xs, ys, zs, c='b')

	colors = ["r", "b", "g"]
	#print contact points of first phase
	for id_c, points in enumerate(p):
		xs = [point[0] for point in points]
		ys = [point[1] for point in points]
		zs = [point[2] for point in points]
		ax.scatter(xs, ys, zs, c=colors[id_c])
		
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

	plt.show()
	return var_final, params
	
def solve_com_RRT(fullBody, states, state_id, computeCones = False, mu = 1, dt =0.1, reduce_ineq = True, num_optims = 0, draw = False, verbose = False, limbsCOMConstraints = None):
	print "callgin gen ",state_id
	if(draw):
		res = draw_trajectory(fullBody, states, state_id, computeCones, mu, dt, reduce_ineq, verbose, limbsCOMConstraints)		
	else:
		res = gen_trajectory(fullBody, states, state_id, computeCones, mu, dt, reduce_ineq, verbose, limbsCOMConstraints)
	t = res[1]["t_init_phases"];
	dt = res[1]["dt"];
	final_state = res[0]
	c0 =  res[1]["x_init"][0:3]
	comPos = [c0] + [c.tolist() for c in final_state['c']]
	comPosPerPhase = [[comPos[(int)(t_id/dt)] for t_id in np.arange(t[index],t[index+1],dt)] for index, _ in enumerate(t[:-1]) ]
	comPosPerPhase[-1].append(comPos[-1])
	assert(len(comPos) == len(comPosPerPhase[0]) + len(comPosPerPhase[1]) + len(comPosPerPhase[2]))
	#~ assert(comPos == [item for sublist in comPosPerPhase for item in sublist])
	print "done. generating state path ",state_id
	
	return fullBody.comRRTFromPos(state_id,comPosPerPhase[0],comPosPerPhase[1],comPosPerPhase[2],num_optims)
	

	
