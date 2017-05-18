#do the loading of the obj file
import numpy as np
from collections import namedtuple
ObjectData = namedtuple("ObjectData", "V T N F")
Inequalities = namedtuple("Inequality", "A b N V")

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
	# last var is v because we need it
	return [n[0], n[1], n[2], np.array(v).dot(np.array(n))]
	
def as_inequalities(obj):
	#for each face, find first three points and deduce plane
	#inequality is given by normal
	A= np.empty([len(obj.F), 3])
	b = np.empty(len(obj.F))
	V = np.ones([len(obj.F), 4])
	N = np.empty([len(obj.F), 3])
	for f in range(0, len(obj.F)):
		face = obj.F[f]
		v = obj.V[face[0][0]]
		# assume normals are in obj
		n = obj.N[face[0][2]]
		ineq = inequality(v,n)
		A[f,:] = ineq[0:3]
		b[f] = ineq[3]
		V[f,0:3] = v
		N[f,:] = n
	return Inequalities(A,b, N, V)
	
def is_inside(inequalities, pt):
	return ((inequalities.A.dot(pt) - inequalities.b) < 0).all()

#~ def rotate_inequalities_q():

# TODO this is naive, should be a way to simply update d
def rotate_inequalities(ineq, transform):
	#for each face, find first three points and deduce plane
	#inequality is given by normal
	A = np.empty([len(ineq.A), 3])
	b = np.empty(len(ineq.b))
	V = np.ones([len(ineq.V), 4])
	N = np.ones([len(ineq.N), 3])
	for i in range(0, len(b)):
		v = transform.dot(ineq.V[i,:])
		n = transform[0:3,0:3].dot(ineq.N[i,0:3])
		ine = inequality(v[0:3],n[0:3])
		A[i,:] = ine[0:3]
		b[i] = ine[3]
		V[i,:] = v
		N[i,:] = n
	return Inequalities(A,b, N, V)

from pickle import dump
def ineq_to_file(ineq, filename):
	f1=open(filename, 'w+')
	res = { 'A' : ineq.A, 'b' : ineq.b, 'N' : ineq.N, 'V' : ineq.V}
	dump(res, f1)
	f1.close()
	
from pickle import load
def ineq_from_file(filename):
	f1=open(filename, 'r')
	res = load(f1)
	return Inequalities(res['A'], res['b'],res['N'],res['V'])
	
def test_inequality():
	n = np.array([0,-1,0])
	v = np.array([0,1,1])
	if inequality(v,n) != [0,-1,0,-1]:
		print("error in test_inequality")
	else:
		print("test_inequality successful")

def __gen_data():
	obj = load_obj('./hrp2/RL_com._reduced.obj')
	ineq = as_inequalities(obj)
	ok_points = [[0,0,0], [0.0813, 0.0974, 0.2326], [-0.3387, 0.1271, -0.5354]]
	not_ok_points = [[-0.3399, 0.2478, -0.722],[-0.1385,-0.4401,-0.1071]]
	return obj, ineq, ok_points, not_ok_points

def test_belonging():
	data = __gen_data()
	ineq = data[1]
	ok_points = data[2]
	not_ok_points = data[3]
	for p in ok_points:
		assert (is_inside(ineq, np.array(p))), "point " + str(p) + " should be inside object"
	for p in not_ok_points:
		assert (not is_inside(ineq, np.array(p))), "point " + str(p) + " should NOT be inside object"
	print("test_belonging successful")
	
def test_rotate_inequalities():
	
	tr = np.array([[ 1.        ,  0.        ,  0.        ,  0.        ],
				   [ 0.        ,  0.98006658, -0.19866933,  2.        ],
				   [ 0.        ,  0.19866933,  0.98006658,  0.        ],
				   [ 0.        ,  0.        ,  0.        ,  1.        ]])
	
	data = __gen_data()
	ineq = rotate_inequalities(data[1], tr)
	ok_points =  [tr.dot(np.array(el + [1]))[0:3] for el in data[2]]
	not_ok_points = [tr.dot(np.array(el + [1]))[0:3] for el in data[3]]
	for p in ok_points:
		assert (is_inside(ineq, p)), "point " + str(p) + " should be inside object"
	for p in not_ok_points:
		assert (not is_inside(ineq, p)), "point " + str(p) + " should NOT be inside object"
	print("test_rotate_inequalities successful")
	

def load_obj_and_save_ineq(in_name, out_name):
	obj = load_obj(in_name)
	ineq = as_inequalities(obj)
	ineq_to_file (ineq, out_name)	
	
load_obj_and_save_ineq('./lfleg_com_reduced.obj','./lfleg_com.ineq')
load_obj_and_save_ineq('./lhleg_com_reduced.obj','./lhleg_com.ineq')
load_obj_and_save_ineq('./rhleg_com_reduced.obj','./rhleg_com.ineq')
load_obj_and_save_ineq('./rfleg_com_reduced.obj','./rfleg_com.ineq')
