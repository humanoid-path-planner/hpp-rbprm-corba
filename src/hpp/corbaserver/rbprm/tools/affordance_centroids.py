from numpy import array, cross
from numpy.linalg import norm


## tools to retrive obstacles of a given affordance, indexed by the centroid of their surface.
## method to import is computeAffordanceCentroids

def flat(pts):
    return [item for sublist in pts for item in sublist]

__EPS = 1e-5

def __filter_points(points):
    res = []
    for el in points:
        el_arr = array(el)
        add = True
        for al in res:
            if(norm(al - el_arr) < __EPS):
                add = False
                break
        if add:
            res += [array(el)]
    return res

def __normal(points):
    p1 = array(points[0])
    p2 = array(points[1])
    p3 = array(points[2])
    normal = cross((p2 - p1),(p3 - p1))
    normal /= norm(normal)
    return normal.tolist()
    
def __centroid(points):
    return sum(points) / len(points)

def __centroid_list(list_points):
    return [[__centroid(__filter_points(flat(pts))).tolist(), __normal(pts[0]) ]  for pts in list_points]

def computeAffordanceCentroids(afftool, affordances=["Support","Lean"]):
    all_points = []
    for _, el in enumerate(affordances):
        all_points += afftool.getAffordancePoints(el)
    return __centroid_list(all_points)

b_id = 0

def draw_centroid(gui, winId, pt, scene="n_name", color = [1,1,1,0.3]):
    p = pt[0]
    n = array(pt[0]) + 0.03 * array(pt[1])
    resolution = 0.01
    global b_id
    boxname = scene+"/"+str(b_id)
    boxnameN = scene+"/"+str(b_id)+"n"
    b_id += 1
    gui.addBox(boxname,resolution,resolution,resolution, color)
    gui.addBox(boxnameN,resolution,resolution,resolution, color)
    gui.applyConfiguration(boxname,[p[0],p[1],p[2],1,0,0,0])
    gui.applyConfiguration(boxnameN,[n[0],n[1],n[2],1,0,0,0])
    gui.addSceneToWindow(scene,winId)
    gui.refresh()

def draw_centroids(gui, winId, pts_lists, scene="n_name", color = [1,0,0,1]):
    gui.createScene(scene)
    for _, pt in enumerate(pts_lists):
        draw_centroid(gui, winId, pt, scene=scene, color = color)
    
