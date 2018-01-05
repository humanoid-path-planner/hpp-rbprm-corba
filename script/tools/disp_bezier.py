from spline import bezier, bezier6, polynom, exact_cubic, curve_constraints, spline_deriv_constraint, from_bezier

from numpy import matrix
from numpy.linalg import norm




def displayBezierCurve(r,curve,step=0.001,color=[0.85, 0.75, 0.15, 1.0],name=None):
    if name==None:
        name="bezier_curve"
        list = r.client.gui.getNodeList()
        i=0
        while list.count(name) > 0:
            name="bezier_curve_"+str(i)
            i+=1
    current=0
    path=[]
    while current<= curve.max():
        path+=[curve(current).transpose().tolist()[0]]
        current +=step
    r.client.gui.addCurve(name,path,color)
    r.client.gui.addToGroup(name,r.sceneName)    
    r.client.gui.refresh()

def displayBezierWaypoints(r,wp,step=0.001,color=[0.85, 0.75, 0.15, 1.0],name=None):
    waypoints=matrix(wp).transpose()
    curve = bezier(waypoints)
    displayBezierCurve(r,curve,step,color,name)
    return curve
