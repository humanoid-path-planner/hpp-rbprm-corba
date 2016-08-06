from cwc import cone_optimization
import numpy as np

def __get_com(robot, config):
	save = robot.getCurrentConfig()
	robot.setCurrentConfig(config)
	com = robot.getCenterOfMass()	
	robot.setCurrentConfig(save)
	return com

def gen_trajectory(fullBody, states, state_id, mu = 1, reduce_ineq = True):
	init_com = __get_com(fullBody, states[state_id])
	end_com = __get_com(fullBody, states[state_id+1])
	p, N = fullBody.computeContactPoints(state_id)
	mass = fullBody.getMass()
	t_end_phases = [0]
	[t_end_phases.append(t_end_phases[-1]+0.5) for _ in range(len(p))]
	print t_end_phases
	dt = 0.1
	return cone_optimization(p, N, [init_com + [0,0,0], end_com + [0,0,0]], t_end_phases[1:], dt, mu, mass, 9.81, reduce_ineq)

def draw_trajectory(fullBody, states, state_id, mu = 1, reduce_ineq = True ):
	var_final, params = gen_trajectory(fullBody, states, state_id, mu , reduce_ineq)
	p, N = fullBody.computeContactPoints(state_id)
	print var_final
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
