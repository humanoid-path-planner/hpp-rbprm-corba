from curves import bezier, polynomial

from numpy import matrix
from numpy.linalg import norm




def displayBezierCurve(r,curve,step=0.001,color=[0.85, 0.75, 0.15, 1.0],name=None,offset = None):
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
        p = curve(current).transpose().tolist()[0]
        if offset:
            assert(len(offset) == len(p))
            for i in range(len(p)):
                p[i] += offset[i]
        path+=[p]
        current +=step
    r.client.gui.addCurve(name,path,color)
    r.client.gui.addToGroup(name,r.sceneName)    
    r.client.gui.refresh()

def displayBezierWaypoints(r,wp,step=0.001,color=[0.85, 0.75, 0.15, 1.0],name=None):
    waypoints=matrix(wp).transpose()
    curve = bezier(waypoints)
    displayBezierCurve(r,curve,step,color,name)
    return curve

def showPath(r,pp,pid):
    if len(pid)==1:
        pp.displayPath(int(pid[0]),color=r.color.red)
        r.client.gui.setVisibility('path_'+str(int(pid[0]))+'_root','ALWAYS_ON_TOP')
    elif len(pid)==4:
        pp.displayPath(int(pid[1]),color=r.color.yellow)
        r.client.gui.setVisibility('path_'+str(int(pid[1]))+'_root','ALWAYS_ON_TOP')  
        pp.displayPath(int(pid[2]),color=r.color.red)
        r.client.gui.setVisibility('path_'+str(int(pid[2]))+'_root','ALWAYS_ON_TOP')  
        pp.displayPath(int(pid[3]),color=r.color.yellow)
        r.client.gui.setVisibility('path_'+str(int(pid[3]))+'_root','ALWAYS_ON_TOP')
    elif len(pid) == 3:
        print("only two phases, not implemented yet.")
    else:
        print("no path, test failed.")


