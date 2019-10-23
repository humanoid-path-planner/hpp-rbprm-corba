# do the loading of the obj file
from collections import namedtuple
from pickle import dump, load

import numpy as np

ObjectData = namedtuple("ObjectData", "V T N F")
Inequalities = namedtuple("Inequality", "A b N V")

__EPS = 0.0


def toFloat(stringArray):
    res = np.zeros(len(stringArray))
    for i in range(0, len(stringArray)):
        res[i] = float(stringArray[i])
    return res


def load_obj(filename):
    V = []  # vertex
    T = []  # texcoords
    N = []  # normals
    F = []  # face indexies

    fh = open(filename)
    for line in fh:
        if line[0] == '#':
            continue

        line = line.strip().split(' ')
        if line[0] == 'v':  # vertex
            V.append(toFloat(line[1:]))
        elif line[0] == 'vt':  # tex-coord
            T.append(line[1:])
        elif line[0] == 'vn':  # normal vector
            N.append(toFloat(line[1:]))
        elif line[0] == 'f':  # face
            face = line[1:]
            for i in range(0, len(face)):
                face[i] = face[i].split('/')
                # OBJ indexies are 1 based not 0 based hence the -1
                # convert indexies to integer
                for j in range(0, len(face[i])):
                    if j != 1:
                        face[i][j] = int(face[i][j]) - 1
            F.append(face)
    fh.close()
    return ObjectData(V, T, N, F)


# find a point such that ax + by + cz = d that is the closest to the origin
def find_point_on_plane(a, b, c, d):
    # project 0 to the plane
    m = np.zeros((4, 4))
    m[:3, :3] = np.identity(3)
    m[:, 3] = [-a, -b, -c, 0]
    m[3, :3] = [a, b, c]
    n = np.zeros(4)
    n[-1] = d
    res = np.linalg.inv(m).dot(n)[:-1]

    return res


# Create an Inequalities object given the inequalities matrices A,b, s.t. Ax <=b
def inequalities_to_Inequalities_object(A, b):
    nrows = A.shape[0]
    V = np.ones([nrows, 4])
    N = np.empty([nrows, 3])
    i = 0
    for ai, bi in zip(A, b):
        N[i, :] = ai
        V[i, :3] = find_point_on_plane(ai[0], ai[1], ai[2], bi)
        i += 1
    return Inequalities(A, b, N, V)


def inequality(v, n):
    # the plan has for equation ax + by + cz = d, with a b c coordinates of the normal
    # inequality is then ax + by +cz -d <= 0
    # last var is v because we need it
    return [n[0], n[1], n[2], np.array(v).dot(np.array(n)) + __EPS]


def as_inequalities(obj):
    # for each face, find first three points and deduce plane
    # inequality is given by normal
    A = np.empty([len(obj.F), 3])
    b = np.empty(len(obj.F))
    V = np.ones([len(obj.F), 4])
    N = np.empty([len(obj.F), 3])
    for f in range(0, len(obj.F)):
        face = obj.F[f]
        v = obj.V[face[0][0]]
        #  assume normals are in obj
        n = obj.N[face[0][2]]
        ineq = inequality(v, n)
        A[f, :] = ineq[0:3]
        b[f] = ineq[3]
        V[f, 0:3] = v
        N[f, :] = n
    return Inequalities(A, b, N, V)


def is_inside(inequalities, pt):
    return ((inequalities.A.dot(pt) - inequalities.b) < 0).all()


# ~ def rotate_inequalities_q():


# TODO this is naive, should be a way to simply update d
def rotate_inequalities(ineq, transform):
    # for each face, find first three points and deduce plane
    # inequality is given by normal
    A = np.empty([len(ineq.A), 3])
    b = np.empty(len(ineq.b))
    V = np.ones([len(ineq.V), 4])
    N = np.ones([len(ineq.N), 3])
    for i in range(0, len(b)):
        v = transform.dot(ineq.V[i, :])
        n = transform[0:3, 0:3].dot(ineq.N[i, 0:3])
        ine = inequality(v[0:3], n[0:3])
        A[i, :] = ine[0:3]
        b[i] = ine[3]
        V[i, :] = v
        N[i, :] = n
    return Inequalities(A, b, N, V)


def ineq_to_file(ineq, filename):
    f1 = open(filename, 'w+')
    res = {'A': ineq.A, 'b': ineq.b, 'N': ineq.N, 'V': ineq.V}
    dump(res, f1)
    f1.close()


def ineq_from_file(filename):
    f1 = open(filename, 'r')
    res = load(f1)
    return Inequalities(res['A'], res['b'], res['N'], res['V'])


def test_inequality():
    n = np.array([0, -1, 0])
    v = np.array([0, 1, 1])
    if inequality(v, n) != [0, -1, 0, -1]:
        print("error in test_inequality")
    else:
        print("test_inequality successful")


def __gen_data():
    import os
    filepath = os.environ[
        'DEVEL_HPP_DIR'] + '/install/share/hrp2-rbprm/com_inequalities/RLEG_JOINT0_com_constraints.obj'
    obj = load_obj(filepath)
    ineq = as_inequalities(obj)
    ok_points = [[0, 0, 0], [0.0813, 0.0974, 0.2326]]
    not_ok_points = [[-0.3399, 0.2478, -0.722], [-0.1385, -0.4401, -0.1071]]
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

    tr = np.array([[1., 0., 0., 0.], [0., 0.98006658, -0.19866933, 2.], [0., 0.19866933, 0.98006658, 0.],
                   [0., 0., 0., 1.]])

    data = __gen_data()
    ineq = rotate_inequalities(data[1], tr)
    ok_points = [tr.dot(np.array(el + [1]))[0:3] for el in data[2]]
    not_ok_points = [tr.dot(np.array(el + [1]))[0:3] for el in data[3]]
    for p in ok_points:
        assert (is_inside(ineq, p)), "point " + str(p) + " should be inside object"
    for p in not_ok_points:
        assert (not is_inside(ineq, p)), "point " + str(p) + " should NOT be inside object"
    print("test_rotate_inequalities successful")


def test_inequalities_to_Inequalities_object():
    data = __gen_data()
    ineq = data[1]

    tr = np.array([[1., 0., 0., 0.], [0., 0.98006658, -0.19866933, 2.], [0., 0.19866933, 0.98006658, 0.],
                   [0., 0., 0., 1.]])
    ok_points = [tr.dot(np.array(el + [1]))[0:3] for el in data[2]]
    not_ok_points = [tr.dot(np.array(el + [1]))[0:3] for el in data[3]]

    ineq2 = inequalities_to_Inequalities_object(ineq.A, ineq.b)
    ineq2 = rotate_inequalities(ineq2, tr)
    for p in ok_points:
        assert (is_inside(ineq2, p)), "point " + str(p) + " should be inside object"
    for p in not_ok_points:
        assert (not is_inside(ineq2, p)), "point " + str(p) + " should NOT be inside object"
    print("test_inequalities_to_Inequalities_object successful")


def load_obj_and_save_ineq(in_name, out_name):
    obj = load_obj(in_name)
    ineq = as_inequalities(obj)
    ineq_to_file(ineq, out_name)


# ~ load_obj_and_save_ineq('./spiderman/LA_com_reduced.obj','./spiderman/LA_com.ineq')
# ~ load_obj_and_save_ineq('./spiderman/RA_com_reduced.obj','./spiderman/RA_com.ineq')
# ~ load_obj_and_save_ineq('./spiderman/LL_com_reduced.obj','./spiderman/LL_com.ineq')
# ~ load_obj_and_save_ineq('./spiderman/RL_com_reduced.obj','./spiderman/RL_com.ineq')

if __name__ == '__main__':
    test_inequality()
    test_belonging()
    test_rotate_inequalities()
    test_inequalities_to_Inequalities_object()
