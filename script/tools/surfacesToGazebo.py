import tools.getSurfaceExtremumPoints as ep
import numpy as np
from pinocchio import Quaternion
from pinocchio.utils import matrixToRpy

NAME_ID = 0
NAME = "box"
WIDTH = 0.02
Z_AXIS = np.matrix([0,0,1]).T

"""
<model name="box_1_model">
      <pose>0.15 0.25 0.145 0 0 0</pose>
      <static>true</static>
      <link name="box_1_body">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.5 0.03</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.5 0.03</size>
            </box>
          </geometry>
        </visual>
      </link>
</model>
"""


def computePoseFromSurface(surface):
  points = surface
  #normal = surface[1]
  normal = [0,0,1]
  center = np.zeros(3)
  for pt in points:
    center += np.array(pt)
  center /= len(points)
  center -= np.array(normal)*(WIDTH/2.)

  # rotation in rpy : 
  q_rot = Quaternion()
  n_m = np.matrix(normal).T
  q_rot.setFromTwoVectors(Z_AXIS,n_m)
  rpy = matrixToRpy(q_rot.matrix())
  pose = center.tolist() + rpy.T.tolist()[0]
  return pose


# ASSUME RECTANGLE ALIGNED WITH X,Y AXIS !!
# FIXME
def computeSizeFromSurface(surface):
  size = [0,0,0]
  size[2] = WIDTH
  points = surface
  for i in range(len(points)-1):
    pt0 = points[i]
    pt1 = points[i+1]
    #print "pt0 = ",pt0
    #print "pt1 = ",pt1
    sX = (abs(pt0[0] - pt1[0]))
    if sX > 0.01:
      size[0] = sX
    sY = (abs(pt0[1] - pt1[1]))
    if sY > 0.01:
      size[1] = sY

  return size

def genStringForSurface(surface):
  pose = computePoseFromSurface(surface)
  size = computeSizeFromSurface(surface)
  global NAME_ID
  res = ""
  res += "<model name=\""+NAME+"_"+str(NAME_ID)+"_model\">\n"
  res += "\t<pose>"+str(pose[0])+" "+str(pose[1])+" "+str(pose[2])+" "+str(pose[3])+" "+str(pose[4])+" "+str(pose[5])+"</pose>\n"
  res += "\t<static>true</static>\n"
  res += "\t<link name=\""+NAME+"_"+str(NAME_ID)+"_body\">\n"
  res += "\t<inertial>\n"
  res += "\t\t<mass>1.0</mass>\n"
  res += "\t\t<inertia>\n"
  res += "\t\t\t<ixx>1.0</ixx>\n"
  res += "\t\t\t<ixy>0.0</ixy>\n"
  res += "\t\t\t<ixz>0.0</ixz>\n"
  res += "\t\t\t<iyy>1.0</iyy>\n"
  res += "\t\t\t<iyz>0.0</iyz>\n"
  res += "\t\t\t<izz>1.0</izz>\n"
  res += "\t\t</inertia>\n"
  res += "\t</inertial>\n"
  res += "\t\t<collision name=\"collision\">\n"
  res += "\t\t<geometry>\n"
  res += "\t\t\t<box>\n"
  res += "\t\t\t\t<size>"+str(size[0])+" "+str(size[1])+" "+str(size[2])+"</size>\n"
  res += "\t\t\t</box>\n"
  res += "\t\t</geometry>\n"
  res += "\t</collision>\n"
  res += "\t<visual name=\"visual\">\n"
  res += "\t\t<geometry>\n"
  res += "\t\t\t<box>\n"
  res += "\t\t\t\t<size>"+str(size[0])+" "+str(size[1])+" "+str(size[2])+"</size>\n"
  res += "\t\t\t</box>\n"
  res += "\t\t</geometry>\n"
  res += "\t</visual>\n"
  res += "\t</link>\n"
  res += "</model>\n"
  NAME_ID += 1
  return res


def getGazeboString(afftool):
  surfaces = ep.contactSurfaces(afftool)
  res = ""
  for surface in surfaces:
    res += genStringForSurface(surface)
    res += "\n\n"

  return res



