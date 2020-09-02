import numpy as np
# from hpp.corbaserver.rbprm.hrp2 import Robot as rob
# from hpp.corbaserver.rbprm.tools.obj_to_constraints import load_obj, as_inequalities, rotate_inequalities
# from hpp_centroidal_dynamics import *
# from hpp_spline import *e
from numpy import array, hstack, identity, matrix, ones, vstack, zeros
from scipy.spatial import ConvexHull

# import eigenpy
import cdd

# from hpp_bezier_com_traj import *
# from qp import solve_lp

Id = matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
z = array([0., 0., 1.])
zero3 = zeros(3)


def generators(A, b, Aeq=None, beq=None):
    m = np.hstack([b, -A])
    matcdd = cdd.Matrix(m)
    matcdd.rep_type = cdd.RepType.INEQUALITY

    if Aeq is not None:
        meq = np.hstack([beq, -Aeq])
        matcdd.extend(meq.tolist(), True)

    H = cdd.Polyhedron(matcdd)
    g = H.get_generators()

    return [array(g[el][1:]) for el in range(g.row_size)], H


def filter(pts):
    hull = ConvexHull(pts, qhull_options='Q12')
    return [pts[i] for i in hull.vertices.tolist()]


def ineq(pts, canonicalize=False):
    apts = array(pts)
    m = np.hstack([ones((apts.shape[0], 1)), apts])
    matcdd = cdd.Matrix(m)
    matcdd.rep_type = cdd.RepType.GENERATOR
    H = cdd.Polyhedron(matcdd)
    bmA = H.get_inequalities()
    if canonicalize:
        bmA.canonicalize()
    Ares = zeros((bmA.row_size, bmA.col_size - 1))
    bres = zeros(bmA.row_size)
    for i in range(bmA.row_size):
        bmAl = array(bmA[i])
        Ares[i, :] = -bmAl[1:]
        bres[i] = bmAl[0]
    return Ares, bres


def ineqQHull(hull):
    A = hull.equations[:, :-1]
    b = -hull.equations[:, -1]
    return A, b


def canon(A, b):
    m = np.hstack([b, -A])
    matcdd = cdd.Matrix(m)
    matcdd.rep_type = 1
    H = cdd.Polyhedron(matcdd)
    bmA = H.get_inequalities()
    # bmA.canonicalize()
    Ares = zeros((bmA.row_size, bmA.col_size - 1))
    bres = zeros((bmA.row_size, 1))
    for i in range(bmA.row_size):
        # print("line ", array(bmA[i]))
        # print("A ", A[i][:])
        # print("b ", b[i])
        bmAl = array(bmA[i])
        Ares[i, :] = -bmAl[1:]
        bres[i] = bmAl[0]
        # print("Ares ",Ares[i,:])
        # print("bres ",bres[i])
    return Ares, bres


def genPolytope(A, b):
    pts, H = generators(A, b)
    apts = array(pts)
    if len(apts) > 0:
        hull = ConvexHull(apts)
        return hull, pts, apts, H
    return None, None, None, None


def convex_hull_ineq(pts):
    return None
    """
    # TODO: what is cData ?
    m = cData.contactPhase_.getMass()
    # get 6D polytope
    (H, h) = ineqFromCdata(cData)
    #project to the space where aceleration is 0
    D = zeros((6, 3))
    D[3:, :] = m * gX

    d = zeros((6, ))
    d[:3] = -m * g

    A = H.dot(D)
    b = h.reshape((-1, )) - H.dot(d)
    #add kinematic polytope
    (K, k) = (cData.Kin_[0], cData.Kin_[1].reshape(-1, ))

    resA = vstack([A, K])
    resb = concatenate([b, k]).reshape((-1, 1))

    #DEBUG
    allpts = generators(resA, resb)[0]
    error = False
    for pt in allpts:
        print("pt ", pt)
        assert (resA.dot(pt.reshape((-1, 1))) - resb).max() < 0.001, "antecedent point not in End polytope" + str(
            (resA.dot(pt.reshape((-1, 1))) - resb).max())
        if (H.dot(w(m, pt).reshape((-1, 1))) - h).max() > 0.001:
            error = True
            print("antecedent point not in End polytope" + str((H.dot(w(m, pt).reshape((-1, 1))) - h).max()))
    assert not error, str(len(allpts))

    return (resA, resb)
    # return (A, b)
    # return (vstack([A, K]), None)
    """


def default_transform_from_pos_normal(pos, normal):
    # print("pos ", pos
    # print("normal ", normal)
    f = array([0., 0., 1.])
    t = array(normal)
    v = np.cross(f, t)
    c = np.dot(f, t)
    if c > 0.99:
        rot = identity(3)
    else:
        # u = v / norm(v)
        h = (1. - c) / (1. - c**2)

        vx, vy, vz = v
        rot = array([[c + h * vx**2, h * vx * vy - vz, h * vx * vz + vy],
                     [h * vx * vy + vz, c + h * vy**2, h * vy * vz - vx],
                     [h * vx * vz - vy, h * vy * vz + vx, c + h * vz**2]])
    return vstack([hstack([rot, pos.reshape((-1, 1))]), [0., 0., 0., 1.]])


def continuous(h, initpts):
    dic = {}
    pts = []
    for i, pt in enumerate(h.vertices.tolist()):
        pts += [initpts[pt]]
        dic[pt] = i
    faces = []
    for f in h.simplices:
        faces += [[dic[idx] + 1 for idx in f]]
    return pts, faces


def hull_to_obj(h, pts, name):
    pts, faces = continuous(h, pts)
    f = open(name, "w")
    # first write points
    for pt in pts:
        # print("??")
        f.write('v ' + str(pt[0]) + ' ' + str(pt[1]) + ' ' + str(pt[2]) + ' \n')
    f.write('g foo\n')
    for pt in faces:
        # print("???")
        f.write('f ' + str(pt[0]) + ' ' + str(pt[1]) + ' ' + str(pt[2]) + ' \n')
    f.write('g \n')
    f.close()


# function vertface2obj(v,f,name)
# % VERTFACE2OBJ Save a set of vertice coordinates and faces as a Wavefront/Alias Obj file
# % VERTFACE2OBJ(v,f,fname)
# %     v is a Nx3 matrix of vertex coordinates.
# %     f is a Mx3 matrix of vertex indices.
# %     fname is the filename to save the obj file.

# fid = fopen(name,'w');

# for i=1:size(v,1)
# fprintf(fid,'v %f %f %f\n',v(i,1),v(i,2),v(i,3));
# end

# fprintf(fid,'g foo\n');

# for i=1:size(f,1);
# fprintf(fid,'f %d %d %d\n',f(i,1),f(i,2),f(i,3));
# end
# fprintf(fid,'g\n');

# fclose(fid);
