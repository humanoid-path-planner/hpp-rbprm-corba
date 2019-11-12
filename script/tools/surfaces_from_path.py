from numpy import arange, array
from pinocchio import Quaternion,SE3,XYZQUATToSe3
from tools.narrow_convex_hull import getSurfaceExtremumPoints, removeDuplicates, normal, area
from tools.display_tools import displaySurfaceFromPoints
import numpy as np
from pinocchio import Quaternion, log3
import eigenpy
eigenpy.switchToNumpyMatrix()

ROBOT_NAME = 'talos'
MAX_SURFACE = 5. # if a contact surface is greater than this value, the intersection is used instead of the whole surface
LF = 0
RF = 1  



# change the format into an array  
def listToArray (seqs):
  nseq = []; nseqs= []
  for seq in seqs:
    nseq = []
    for surface in seq:
      nseq.append(array(surface).T)
    nseqs.append(nseq)
  return nseqs

# get configurations along the path  
def getConfigsFromPath (ps, pathId = 0, discretisationStepSize = 1.) :
  configs = []
  pathLength = ps.pathLength(pathId)
  for s in arange (0, pathLength, discretisationStepSize) :
    configs.append(ps.configAtParam(pathId, s))
  return configs

# get all the contact surfaces (pts and normal)
def getAllSurfaces(afftool) :
  l = afftool.getAffordancePoints("Support")
  return [(getSurfaceExtremumPoints(el), normal(el[0])) for el in l]
    
# get surface information
def getAllSurfacesDict (afftool) :
  all_surfaces = getAllSurfaces(afftool) 
  all_names = afftool.getAffRefObstacles("Support") # id in names and surfaces match
  surfaces_dict = dict(zip(all_names, all_surfaces)) # map surface names to surface points
  return surfaces_dict

# get rotation matrices form configs
def getRotationMatrixFromConfigs(configs) :
  eigenpy.switchToNumpyMatrix()
  R = []
  for config in configs:
    q = [0,0,0] + config[3:7]
    #print "q = ",q
    placement = XYZQUATToSe3(q)
    #print "placement = ",placement
    rot = placement.rotation
    #print "rot = ",rot
    R.append(np.array(rot))
  #print "R in getRotationMatrixFromConfigs : ",R
  return R   
    
# get contacted surface names at configuration
def getContactsNames(rbprmBuilder,i,q):
  if i % 2 == LF : # left leg 
    step_contacts = rbprmBuilder.clientRbprm.rbprm.getCollidingObstacleAtConfig(q, ROBOT_NAME + '_lleg_rom') 
  elif i % 2 == RF : # right leg 
    step_contacts = rbprmBuilder.clientRbprm.rbprm.getCollidingObstacleAtConfig(q, ROBOT_NAME + '_rleg_rom')
  return step_contacts

# get intersections with the rom and surface at configuration
def getContactsIntersections(rbprmBuilder,i,q):
  if i % 2 == LF : # left leg
    intersections = rbprmBuilder.getContactSurfacesAtConfig(q, ROBOT_NAME + '_lleg_rom') 
  elif i % 2 == RF : # right leg
    intersections = rbprmBuilder.getContactSurfacesAtConfig(q, ROBOT_NAME + '_rleg_rom')
  return intersections

# merge phases with the next phase
def getMergedPhases (seqs):
  nseqs = []
  for i, seq in enumerate(seqs):
    nseq = []
    if i == len(seqs)-1: nseq = seqs[i]
    else: nseq = seqs[i]+seqs[i+1]
    nseq = removeDuplicates(nseq)
    nseqs.append(nseq)  
  return nseqs    


def computeRootYawAngleBetwwenConfigs(q0,q1):
  quat0 = Quaternion(q0[6],q0[3],q0[4],q0[5])
  quat1 = Quaternion(q1[6],q1[3],q1[4],q1[5])
  v_angular = log3(quat0.matrix().T*quat1.matrix())
  #print "q_prev : ",q0
  #print "q      : ",q1
  #print "v_angular = ",v_angular
  return v_angular[2,0]

def isYawVariationsInsideBounds(q0,q1,max_yaw = 0.5):
  yaw = abs(computeRootYawAngleBetwwenConfigs(q0,q1))
  #print "yaw = ",yaw
  return  yaw < max_yaw

def getSurfacesFromGuideContinuous(rbprmBuilder,ps,afftool,pId,viewer = None,step = 1.,useIntersection= False,mergeCandidates = False,max_yaw = 0.5):
  pathLength = ps.pathLength(pId) #length of the path
  discretizationStep = 0.01 # step at which we check the colliding surfaces
  #print "path length = ",pathLength
  # get surface information
  all_surfaces = getAllSurfaces(afftool) 
  all_names = afftool.getAffRefObstacles("Support") # id in names and surfaces match
  surfaces_dict = dict(zip(all_names, all_surfaces)) # map surface names to surface points
  seqs = [] # list of list of surfaces : for each phase contain a list of surfaces. One phase is defined by moving of 'step' along the path
  configs = []
  t = -discretizationStep
  current_phase_end = step
  end = False
  i = 0
  q_prev = ps.configAtParam(pId, 0)
  q = q_prev[::]
  configs.append(q)
  while not end: # for all the path
    #print "Looking for surfaces for phase "+str(len(seqs))+" for t in ["+str(t+discretizationStep)+" ; "+str(current_phase_end)+" ] "
    phase_contacts_names = []
    rot_valid = True
    while t < current_phase_end and rot_valid: # get the names of all the surfaces that the rom collide while moving from current_phase_end-step to current_phase_end
      t += discretizationStep
      q = ps.configAtParam(pId, t)
      if not isYawVariationsInsideBounds(q_prev,q,max_yaw):
        #print "yaw variation out of bounds, try to reduce the time step : "
        rot_valid = False
        t -= discretizationStep
        q = ps.configAtParam(pId, t)
        while isYawVariationsInsideBounds(q_prev,q,max_yaw):
          t += 0.0001
          q = ps.configAtParam(pId, t)
      #print " t in getSurfacesFromGuideContinuous : ",t
      step_contacts = getContactsNames(rbprmBuilder,i,q)
      for contact_name in step_contacts : 
        if not contact_name in phase_contacts_names:
          phase_contacts_names.append(contact_name)
    # end current phase
    # get all the surfaces from the names and add it to seqs: 
    if useIntersection : 
      intersections = getContactsIntersections(rbprmBuilder,i,q)
    phase_surfaces = []
    for name in phase_contacts_names:
      surface = surfaces_dict[name][0]
      if useIntersection and area(surface) > MAX_SURFACE : 
        if name in step_contacts : 
          intersection = intersections[step_contacts.index(name)]
          if len(intersection) > 3 :
            phase_surfaces.append(intersection)
      else :
        phase_surfaces.append(surface) # [0] because the last vector contain the normal of the surface
    #print "There was "+str(len(phase_contacts_names))+" surfaces in contact during this phase."
    phase_surfaces = sorted(phase_surfaces) # why is this step required ? without out the lp can fail
    phase_surfaces_array = [] # convert from list to array, we cannot do this before because sorted() require list
    for surface in phase_surfaces:
      phase_surfaces_array.append(array(surface).T)
      if viewer:
        displaySurfaceFromPoints(viewer,surface,[0,0,1,1])
    #print "phase_surfaces_array = ",phase_surfaces_array    
    seqs.append(phase_surfaces_array)
    # increase values for next phase
    q_prev = q[::]
    configs.append(q)
    i += 1 
    if t >= (pathLength - discretizationStep/2.):
      end = True
    t -= discretizationStep # because we want the first iteration of the next phase to test the same t as the last iter of this phase
    current_phase_end = t + step
    if current_phase_end >= pathLength:
      current_phase_end = pathLength
  # end for all the guide path
  #get rotation matrix of the root at each discretization step
  R = getRotationMatrixFromConfigs(configs)
  return R,seqs


def getSurfacesFromPath(rbprmBuilder, configs, surfaces_dict, viewer = None, useIntersection = False, useMergePhase = False):
  seqs = [] 
  # get sequence of surface candidates at each discretization step
  for i, q in enumerate(configs):    
    seq = [] 
    intersections = getContactsIntersections(rbprmBuilder,i,q) # get intersections at config
    phase_contacts_names = getContactsNames(rbprmBuilder,i,q) # get the list of names of the surface in contact at config        
    for j, intersection in enumerate(intersections):
      if useIntersection and area(intersection) > MAX_SURFACE : # append the intersection
        seq.append(intersection) 
      else:
        if len(intersections) == len(phase_contacts_names): # in case getCollidingObstacleAtConfig does not work (because of the rom?)
          seq.append(surfaces_dict[phase_contacts_names[j]][0]) # append the whole surface
        else: seq.append(intersection) # append the intersection
      if viewer:
        displaySurfaceFromPoints(viewer,intersection,[0,0,1,1])
    seqs.append(seq)
    
  # merge candidates with the previous and the next phase
  if useMergePhase: seqs = getMergedPhases (seqs)
    
  seqs = listToArray(seqs) 
  R = getRotationMatrixFromConfigs(configs)
  return R,seqs
    
