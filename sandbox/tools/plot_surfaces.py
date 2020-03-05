import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

COLORS = ['r','g','b','m','y','c']

def plotSurface (points, ax, plt, color_id = -1):
  xs = np.append(points[0,:], points[0,0]).tolist()
  ys = np.append(points[1,:], points[1,0]).tolist()
  zs = (np.append(points[2,:], points[2,0]) - np.ones(len(xs)) * 0.005 * color_id).tolist()    
  if color_id == -1: ax.plot(xs,ys,zs,'gray')
  else: ax.plot(xs,ys,zs,COLORS[color_id])
  plt.draw()

# draw the scene, all the surfaces
def drawScene(all_surfaces, ax = None):
  if ax is None:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = "3d")
  for surface in all_surfaces:
    plotSurface(np.array(surface[0]).T, ax, plt, -1)
  return ax

# draw contact surfaces, same color for each pahse
def drawContacts(surfaces, ax = None):
  color_id = 0
  if ax is None:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = "3d")
  for surfaces_phase in surfaces:
    for surface in surfaces_phase:
      plotSurface(surface, ax, plt, color_id)
    color_id += 1
    if color_id >= len(COLORS):
      color_id = 0
  plt.show()
  return ax

def draw (surfaces, all_surfaces = None):
  ax = None
  if all_surfaces :
    ax = drawScene(all_surfaces)
  drawContacts(surfaces, ax)
