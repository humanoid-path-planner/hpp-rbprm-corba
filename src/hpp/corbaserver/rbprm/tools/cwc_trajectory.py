from cwc import cone_optimization
import numpy as np

def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()
	print 'debut', com
	print 'debut c ', config[0:3]
	print 'z_diff', config[2] - com[2]
	#~ com = config[0:3]	 #assimilate root and com
	robot.setCurrentConfig(save)
	return com

def gen_trajectory(fullBody, states, state_id, computeCones = False, mu = 1, reduce_ineq = True):
	init_com = __get_com(fullBody, states[state_id])
	end_com = __get_com(fullBody, states[state_id+1])
	p, N = fullBody.computeContactPoints(state_id)
	mass = fullBody.getMass()
	t_end_phases = [0]
	[t_end_phases.append(t_end_phases[-1]+1) for _ in range(len(p))]
	dt = 0.1
	cones = None
	if(computeCones):
		cones = [fullBody.getContactCone(state_id)[0]]
		if(len(p) > 2):
			cones.append(fullBody.getContactIntermediateCone(state_id)[0])
		if(len(p) > len(cones)):
			cones.append(fullBody.getContactCone(state_id+1)[0])
	return cone_optimization(p, N, [init_com + [0,0,0], end_com + [0,0,0]], t_end_phases[1:], dt, cones, mu, mass, 9.81, reduce_ineq)

def draw_trajectory(fullBody, states, state_id, computeCones = False, mu = 1, reduce_ineq = True):
	var_final, params = gen_trajectory(fullBody, states, state_id, computeCones, mu , reduce_ineq)
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
	
def solve_com_RRT(fullBody, states, state_id, computeCones = False, mu = 1, reduce_ineq = True, num_optims = 0):
	res = gen_trajectory(fullBody, configs, state_id, computeCones, 0.5, False)
	c0 = configs[i][0:3]
	pos = [c0] + [c.tolist() for c in res[0]['c']]
	pid = fullBody.generateRootPathStates(pos, configs[i], configs[i+1])
	fullBody.comRRT(i,i+1,pid,num_optims)
