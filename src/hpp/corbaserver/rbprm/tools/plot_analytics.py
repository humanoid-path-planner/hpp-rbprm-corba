import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm


cma = cm.autumn

#~ plt.ion()

## Display a 3d plot of the values computed for a limb database
# where all samples take the maximum value of the octree they belong to
# \param robot FullBody object
# \param valueName name of the plotted analytics
# \param limb name of the considered limb
def plotcube(plt, ax, c, pos):
	x = pos[0];	y = pos[1];	z = pos[2]
	r = (float)(pos[3])/2
	
	x1 = [x - r, x + r]
	y1 = [y - r, y + r]
	z1 = [z -r, z - r]
	X, Y = np.meshgrid(x1, y1)
	ax.plot_surface(X,Y,z1, color = cma(c))

	x1 = [x - r, x + r]
	y1 = [y - r, y + r]
	z1 = [z + r, z + r]
	X, Y = np.meshgrid(x1, y1)
	ax.plot_surface(X,Y,z1, color = cma(c))

	x1 = [x - r, x + r]
	y1 = [y + r, y + r]
	z1 = [z + r, z - r]
	X, Z = np.meshgrid(x1, z1)
	ax.plot_surface(X,y1,Z, color = cma(c))

	x1 = [x - r, x + r]
	y1 = [y - r, y - r]
	z1 = [z + r, z - r]
	X, Z = np.meshgrid(x1, z1)
	ax.plot_surface(X,y1,Z, color = cma(c))

	x1 = [x - r, x - r]
	y1 = [y - r, y + r]
	z1 = [z + r, z - r]
	Y, Z = np.meshgrid(y1, z1)
	ax.plot_surface(x1,Y,Z, color = cma(c))

	x1 = [x + r, x + r]
	y1 = [y - r, y + r]
	z1 = [z + r, z - r]
	Y, Z = np.meshgrid(y1, z1)
	ax.plot_surface(x1,Y,Z, color = cma(c))

def getOctreeValues(robot, valueName, limb):
	res = {}
	res ['boxes']  = []
	res ['values'] = []
	octreeIds = robot.getOctreeNodeIds(limb)
	for ocId in octreeIds:
		sampleIds = robot.getSamplesIdsInOctreeNode(limb, ocId)
		max_val = 0;
		for sId in sampleIds:
			i = int(sId)
			g = robot.getSampleValue(limb, valueName, i)
			max_val = max(max_val, g)
		box = robot.getOctreeBox(limb, (int)(ocId))
		res['boxes'].append(box)
		res['values'].append(max_val)
	return res
		

def plotOctreeValues(robot, valueName, limb):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	boxesValues = getOctreeValues(robot, valueName, limb)
	boxes = boxesValues['boxes']
	values = boxesValues['values']
	for i in range(0,len(boxes)):
		plotcube(plt,ax,values[i], boxes[i])
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')
	plt.title(valueName)
	plt.show()

def plotOctreeValuesCompare(ax, boxesValues):
	boxes = boxesValues['boxes']
	values = boxesValues['values']
	for i in range(0,len(boxes)):
		plotcube(plt,ax,values[i], boxes[i])
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

def compareOctreeValues(robotName1, robotName2, boxesValues1, boxesValues2, valueName):
	fig = plt.figure()
	fig.suptitle(valueName)
	ax = fig.add_subplot(121, projection='3d')
	ax.set_title(robotName1)
	plotOctreeValuesCompare(ax, boxesValues1)
	bx = fig.add_subplot(122, projection='3d')
	bx.set_title(robotName2)
	plotOctreeValuesCompare(bx, boxesValues2)
	#~ plt.title(valueName)
	plt.savefig(valueName+'.png')
	plt.draw()

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
		g = robot.getSampleValue(limb, valueName, i)
		pos = robot.getSamplePosition(limb, i)
		xs.append(pos[0])
		ys.append(pos[1])
		zs.append(pos[2])
		g = robot.getSampleValue(limb, valueName, i)
		vals.append(cma(g))
	ax.scatter(xs, ys, zs, color=vals)
	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')	
	plt.title(valueName)
	plt.show()
