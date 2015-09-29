#do the loading of the obj file
import numpy as np
from collections import namedtuple
ObjectData = namedtuple("ObjectData", "V T N F")

def toFloat(stringArray):
	res= np.zeros(len(stringArray))
	for i in range(0,len(stringArray)):
		res[i] = float(stringArray[i])
	return res

def load_obj(filename) :
 V = [] #vertex
 T = [] #texcoords
 N = [] #normals
 F = [] #face indexies

 fh = open(filename)
 for line in fh :
  if line[0] == '#' : continue

  line = line.strip().split(' ')
  if line[0] == 'v' : #vertex
   V.append(toFloat(line[1:]))
  elif line[0] == 'vt' : #tex-coord
   T.append(line[1:])
  elif line[0] == 'vn' : #normal vector
   N.append(toFloat(line[1:]))
  elif line[0] == 'f' : #face
   face = line[1:]
   for i in range(0, len(face)) :
    face[i] = face[i].split('/')
    # OBJ indexies are 1 based not 0 based hence the -1
    # convert indexies to integer
    for j in range(0, len(face[i])): 
		if j!=1:
			face[i][j] = int(face[i][j]) - 1
   F.append(face)

 return ObjectData(V, T, N, F)
 
def inequality(v, n): 
	#the plan has for equation ax + by + cz = d, with a b c coordinates of the normal
	#inequality is then ax + by +cz -d <= 0
	return [n[0], n[1], n[2], -np.array(v).dot(np.array(n))]
	
def asInequalities(obj):
	#for each face, find first three points and deduce plane
	#inequality is given by normal
	res=[]
	for f in range(0, len(obj.F)):
		face = obj.F[f]
		v = obj.V[face[0][0]]
		# assume normals are in obj
		n = obj.N[face[0][2]]
		res.append(inequality(v,n))
	return np.array(res)
	
def is_inside(inequalities, pt):
	p4 = [pt[0], pt[1], pt[2], 1]
	for i in range(0, len(inequalities)):
		if inequalities[i].dot(p4) > 0:
			return 0
	return 1


def test_inequality():
	n = np.array([0,-1,0])
	v = np.array([0,1,1])
	print(inequality(v,n))
	if inequality(v,n) != [0,-1,0,1]:
		print("error in inequality")
	else:
		print("test of inequality successful")

#~ obj = load_obj('../data/roms/comlArmSimplified.obj')
#~ ineq = asInequalities(obj)
#~ print(is_inside(ineq, [0,0,0])) # in
#~ print(is_inside(ineq, [0.873,-0.03551,-0.1630])) #out
#~ print(is_inside(ineq, [0.6730,-0.03551,-0.1630])) #in
#~ print(is_inside(ineq, [-0.08897,0.7172,-0.2494])) #in
