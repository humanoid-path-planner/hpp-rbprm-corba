import numpy as np
from numpy import array, cross
from numpy.linalg import norm

from scipy.spatial import ConvexHull


def normal(points):
    p1 = array(points[0])
    p2 = array(points[1])
    p3 = array(points[2])
    normal = cross((p2 - p1), (p3 - p1))
    normal /= norm(normal)
    return normal.tolist()


def area(s):
    area = 0
    for i in range(1, len(s) - 1):
        area += abs(s[i][0] * (s[i + 1][1] - s[i - 1][1]))
    i = len(s) - 1
    area += abs(s[i][0] * (s[0][1] - s[i - 1][1]))
    return area * 0.5


def cutList2D(lst):
    return [el[:2] for el in lst]


def roundPoints(points, precision):
    return [[round(x, precision) for x in p] for p in points]


def removeDuplicates(points):
    pList = []
    for p in points:
        if p not in pList:
            pList.append(p)
    return pList


def computeAxisAngleRotation(u, c):
    ux = u[0]
    uy = u[1]
    uz = u[2]
    s = np.sqrt(1 - c * c)
    return [
        [c + ux * ux * (1 - c), ux * uy * (1 - c) - uz * s, ux * uz * (1 - c) + uy * s],
        [uy * ux * (1 - c) + uz * s, c + uy * uy * (1 - c), uy * uz * (1 - c) - ux * s],
        [uz * ux * (1 - c) - uy * s, uz * uy * (1 - c) + ux * s, c + uz * uz * (1 - c)],
    ]


def getSurfaceRotation(surface):
    # n = surface[1]
    cosx = np.dot(surface[1], [0, 0, 1])
    axisx = np.cross(surface[1], [0, 0, 1])
    n_axisx = norm(axisx)
    if n_axisx > 0:
        axisx /= n_axisx
    return computeAxisAngleRotation(axisx, cosx)


def getPtsRotation(points):
    return getSurfaceRotation((points, normal(points)))


def getSurfaceTranslation(surface):
    return [sum(x) / len(x) for x in zip(*surface[0])]


def getPtsTranslation(points):
    return getSurfaceTranslation((points, normal(points)))


def allignSurface(surface):
    R = getSurfaceRotation(surface)
    t = getSurfaceTranslation(surface)
    translatedPts = [(array(p) - array(t)).tolist() for p in surface[0]]
    rotatedPts = [np.dot(R, p).tolist() for p in translatedPts]
    return [(array(p) + array(t)).tolist() for p in rotatedPts]


def allignPoints(points):
    return allignSurface((points, normal(points)))


def pointsTransform(points, R, t):
    translatedPts = [(array(pt) - array(t)).tolist() for pt in points]
    rotatedPts = [np.dot(R, pt).tolist() for pt in translatedPts]
    return [(array(pt) + array(t)).tolist() for pt in rotatedPts]


def getSurfaceExtremumPoints(el):
    pts = removeDuplicates(
        el[0] + el[1]
    )  # This might only works in rectangular shape? with triangular mesh*2
    apts = allignPoints(pts)
    hull = ConvexHull(cutList2D(apts))
    return [pts[idx] for idx in hull.vertices]
