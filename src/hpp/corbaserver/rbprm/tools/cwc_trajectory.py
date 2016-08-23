from cwc import cone_optimization
import numpy as np

def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()
	robot.setCurrentConfig(save)
	return com

def gen_trajectory(fullBody, states, state_id, computeCones = False, mu = 1, dt=0.2, reduce_ineq = True, verbose = False):
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
	return cone_optimization(p, N, [init_com + [0,0,0], end_com + [0,0,0]], t_end_phases[1:], dt, cones, mu, mass, 9.81, reduce_ineq, verbose)

def draw_trajectory(fullBody, states, state_id, computeCones = False, mu = 1,  dt=0.2, reduce_ineq = True, verbose = False):
	var_final, params = gen_trajectory(fullBody, states, state_id, computeCones, mu , dt, reduce_ineq, verbose)
	p, N = fullBody.computeContactPoints(state_id)
	import numpy as np
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
	
def solve_com_RRT(fullBody, states, state_id, computeCones = False, mu = 1, dt =0.1, reduce_ineq = True, num_optims = 0, draw = False, verbose = False):
	print "callgin gen ",state_id
	if(draw):
		res = draw_trajectory(fullBody, states, state_id, computeCones, mu, dt, reduce_ineq, verbose)		
	else:
		res = gen_trajectory(fullBody, states, state_id, computeCones, mu, dt, reduce_ineq, verbose)
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
	
	#~ pid = fullBody.generateRootPathStates(comPos, states[state_id], states[state_id+1])
	#~ print "done. shortcuting ",state_id	
	#~ 
	#~ return fullBody.comRRT(state_id,state_id+1,pid,num_optims)

	
