import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
#~ 
#~ def randrange(n, vmin, vmax):
    #~ return (vmax - vmin)*np.random.rand(n) + vmin
#~ 
#~ fig = plt.figure()
#~ ax = fig.add_subplot(111, projection='3d')
#~ n = 100
#~ for c, m, zl, zh in [('r', 'o', -50, -25), ('b', '^', -30, -5)]:
    #~ xs = randrange(n, 23, 32)
    #~ ys = randrange(n, 0, 100)
    #~ zs = randrange(n, zl, zh)
    #~ ax.scatter(xs, ys, zs, c=c, marker=m)
#~ 
#~ ax.set_xlabel('X Label')
#~ ax.set_ylabel('Y Label')
#~ ax.set_zlabel('Z Label')
#~ 
#~ plt.show()

## Display a 3d plot of the values computed for a limb database
#
# \param robot FullBody object
# \param valueName name of the plotted analytics
# \param limb name of the considered limb
def plotValues(robot, valueName, limb):
	xs = []
	ys = []
	zs = []
	vals = []
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	numSamples = robot.getNumSamples(limb)	
	for i in range(0,numSamples):
		pos = robot.getSamplePosition(limb, i)
		xs.append(pos[0])
		ys.append(pos[1])
		zs.append(pos[2])
		g = robot.getSampleValue(limb, valueName, i)
		print g
		vals.append([1-g,g,0])
		#~ ax.scatter(pos[0], pos[1], pos[2], c=robot.getSampleValue(limb, valueName, i))
	ax.scatter(xs, ys, zs, c=vals)
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')
	plt.show()
