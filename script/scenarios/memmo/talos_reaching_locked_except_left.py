from hpp.gepetto import Viewer
from tools.display_tools import *
from hpp.corbaserver import ProblemSolver
import time
from math import sqrt


import argparse, numpy as np
from hpp import Quaternion, Transform
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import loadServerPlugin, createContext

from hpp.corbaserver.robot import Robot, HumanoidRobot


from tools.display_tools import *

HumanoidRobot.packageName = "talos_data"
HumanoidRobot.urdfName = "talos"
HumanoidRobot.urdfSuffix = "_reduced"
HumanoidRobot.srdfSuffix = ""
rootJointType = "freeflyer"


robot = HumanoidRobot("talos", rootJointType)
robot.leftAnkle = "leg_left_6_joint"
robot.rightAnkle = "leg_right_6_joint"
robot.setJointBounds("root_joint", [-0, 2, -2, 2, 0, 2])
robot.setJointBounds("root_joint", [-0., 0.2, -0.64, -0.66, 1., 1.1])
robot.setJointBounds("root_joint", [-0.3, 0.3, -2, 2, 1., 1.1])

ps = ProblemSolver(robot)
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

#init config 

# ~ init_conf = [0, 0, 0, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# ~ init_conf = robot.getCurrentConfig()
init_conf = [0, 0, 0, 0, 0, 0, 1, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, 0, 0.25, -0.173046, 0, 0, 0, 0, -0.25847, -0.173046, 0.0002, -0.]
init_conf[0:7] = [-0.1, -0.65, 1.0192720229567027, 0, 0, sqrt(2) / 2, sqrt(2) / 2]  # root_joint
# ~ init_conf[0:7] = [-0.1, -1.65, 1.0192720229567027, 0, 0, sqrt(2) / 2, sqrt(2) / 2]  # root_joint

#~ ps.resetConstraints()

ps.createOrientationConstraint(
        "waist_yaw", "", "root_joint", (0, 0, 0, 1), [True, True, True] )
    
ps.addPartialCom("talos", ["root_joint"])


# ~ ps.client.problem.createConfigurationConstraint( "cost", init_conf, [0 for _ in range(21)] + [1,1,1] + [0 for _ in range(5)] + [1,1,1] + [0 for _ in range(6)])
# ~ weights = [0 for _ in range(21)] + [1,1,1] + [0 for _ in range(5)] + [1,1,1] + [0 for _ in range(6)]
weights = [0 for _ in range(21)] + [0,0,0] + [0 for _ in range(5)] + [0,0,0] + [0 for _ in range(6)]
weights[3:7] = [1 for _ in range(4)]
print ("weights ", len(weights))
ps.client.problem.createConfigurationConstraint( "cost", init_conf, [0 for _ in range(21)] + [1,1,1] + [0 for _ in range(5)] + [1,1,1] + [0 for _ in range(6)])

robot.createStaticStabilityConstraint(
        "balance", "talos", robot.leftAnkle, robot.rightAnkle, init_conf)
    
def addCons(ps):
    # ~ pass
    #NOTHING !
    # ~ ps.addNumericalConstraints('c',['balancerelative-com', 'balancepose-left-foot', 'balancepose-right-foot'])
    # ~ ps.addNumericalConstraints('c',['balancepose-left-foot', 'balancepose-right-foot'])
    ps.addLockedJointConstraints("locked",lockedJoints)

    

posIdx = 0

def genTarget(ps, effector, position = None, orientation = None, rand = False):
    global posIdx
    ok = False
    while not ok:
        # ~ print ("try")
        ps.resetConstraints()
        #~ v(q)
        p = position
        
        if rand:
            q = robot.shootRandomConfig()
            q [:7] = init_conf[:7]
        
        if position is None:       
            q = robot.shootRandomConfig()
            q [:7] = init_conf[:7]
            robot.setCurrentConfig(q)
            p = robot.getJointPosition(effector) [:3]
        # ~ q = robot.shootRandomConfig()
        # ~ q [:7] = init_conf[:7]
        # ~ robot.setCurrentConfig(q)
        
        #addinx boxes:
        # ~ p[0] = max(p[0],0.4)
        
        robot.setCurrentConfig(init_conf)
        pointName = "point"+str(posIdx)
        pointNameor = pointName+"or"
        ps.createPositionConstraint(
        pointName,
        '',
        effector,
        p,
        (0, 0, 0),
        (True, True, True))
        
        if orientation is not None:
            print ("ORIENTATIONNNNNNNN")
            ps.createOrientationConstraint(
            pointName+"or",
            '',
            effector,
            orientation,
            (True, True, True))
            ps.addNumericalConstraints('pose',[pointName, pointNameor])
        else:
            ps.addNumericalConstraints('pose',[pointName])
        addCons(ps)
        
        res = ps.applyConstraints(init_conf)
        ok = res[0] and robot.isConfigValid(res[1])[0]
        # ~ for i in range(10):
            # ~ if not ok:
                # ~ q = robot.shootRandomConfig()
                # ~ q [:7] = init_conf[:7]
                # ~ res = ps.applyConstraints(q)
                # ~ ok = res[0] and robot.isConfigValid(res[1])[0]
            
        # ~ print ("res [0]", res[0])
        # ~ print ("robot.isConfigValid(res[1])[0]", robot.isConfigValid(res[1])[0])
        if ok:
            # ~ print (ok)
            v(res[1])
        else:
            break
        # ~ q = res[1]
        posIdx += 1
    #~ v(q)
    ps.resetConstraints()
    addCons(ps)
    return res[1], ok


ps.loadPlugin("spline-gradient-based.so")
ps.addPathOptimizer("SimpleTimeParameterization")



#viewer
from hpp.gepetto import ViewerFactory
vf = ViewerFactory (ps)
v = Viewer (ps, displayCoM = True)

#load obstacles
v.loadObstacleModel("gerard_bauzil", "rolling_table", "planning")

xOff = 0.2
yOff = 0.2
zOff = 0.6
# ~ table = [[0.25, 0.3, .7425,0.,0.,0.,1.],[0.25, -0.3, .7425,0.,0.,0.,1.], [-0.25, -0.3, .7425,0.,0.,0.,1.], [-0.25, 0.3, .7425,0.,0.,0.,1.]]
table = [[0.25 + xOff, 0.3+ yOff, .8425 - 0.2,0.,0.,0.,1.],[0.25+ xOff, -0.3- yOff, .8425 - 0.2,0.,0.,0.,1.], [-0.25- xOff, -0.3 - yOff, .8425 - 0.2,0.,0.,0.,1.], [-0.25-xOff, 0.3+ yOff, .8425 - 0.2,0.,0.,0.,1.], 
[0.25 +xOff, 0.3+ yOff, .8425 - 0.2 + zOff,0.,0.,0.,1.],[0.25+ xOff, -0.3-yOff, .8425 - 0.2+ zOff,0.,0.,0.,1.], [-0.25- xOff, -0.3 - yOff, .8425 - 0.2+ zOff,0.,0.,0.,1.], [-0.25- xOff, 0.3+ yOff, .8425 - 0.2+ zOff,0.,0.,0.,1.]]
table = [np.array(el) for el in table]

   
def gen_rand_conf_from_generators(generators):
    size = len(generators)
    weights = np.random.uniform(size = size)
    weights /= sum(weights)    
    target = sum([ w * el  for w,el in zip(weights, generators)])
    return target.tolist()[:3]

q0 = None
q1 = None

def plan(ps, effector, orientation,  generators):
    # ~ q1 = genTarget(ps, effector)
    ok = False
    posT = None
    posT2 = None
    while not ok:
        if generators is not None:
            posT = gen_rand_conf_from_generators(generators)
            q_init, ok = genTarget(ps, effector,posT, orientation, rand = True )
        else:
            q_init, ok = genTarget(ps, effector,None, None, rand = True )
            v(q_init)
            pos = robot.getJointPosition(effector)
            ok = pos[0] < -0.3 and pos[1] > -0.8 and pos[2] < 1. and robot.isConfigValid(q_init)[0]
            if ok:
                pos = robot.getJointPosition(effector)
                print ("init ok ", robot.isConfigValid(q_init))
                print ("init ok ", pos)
                print ("init ok ", pos[2] < 1.)
    ok = False
    while not ok:
        q, ok = genTarget(ps, effector,None, None, rand = True )
        v(q)
        pos = robot.getJointPosition(effector)
        ok = pos[0] > -0.3 and pos[1] > -0.8 and pos[2] < 1. and robot.isConfigValid(q)[0]
        if ok:
            pos = robot.getJointPosition(effector)
            print ("end ok ", pos)
        # ~ posT2 = gen_rand_conf_from_generators(table)
        # ~ q, ok = genTarget(ps, effector,posT2, orientation, rand = True )
    #~ addCons(ps)
    # ~ createBox(posT+[0.,0.,0.,1.])
    # ~ createBox(posT2+[0.,0.,0.,1.])
    # ~ q_init[:7] = init_conf[:7]
    # ~ q[:7] = init_conf[:7]
    print ("init ok ", robot.isConfigValid(q_init))
    ps.setInitialConfig(q_init)
    print ("end ok ", robot.isConfigValid(q))
    ps.addGoalConfig(q)
    global q0, q1
    q0 = q_init
    q1 = q
    v(q0)
    time.sleep(1)
    v(q1)
    time.sleep(1)
    #~ v(q)
    



import os

EXPORT_PATH = "/media/data/memmo/talos_reach0/"
DT = 0.01
# ~ EFFECTORS = ['gripper_left_inner_double_joint','gripper_right_inner_double_joint' ]
EFFECTORS = ['gripper_left_inner_double_joint']
# ~ if not os.path.exists(EXPORT_PATH):
        # ~ os.makedirs(EXPORT_PATH)

START_ZONE_BY_EFFECTORS = [ [[-0.63, -0.54, .48], [-0.23, -0.54, .48], [-0.23, -0.54, 1.4], [-0.63, -0.54, 1.4], [-0.63, -0.2, 1.4], [-0.23, -0.2, 1.4], [-0.63, -0.2, .48], [-0.23, -0.2, .48]  ]  , [ ] ]
START_ZONE_BY_EFFECTORS = [np.array(el) for el in START_ZONE_BY_EFFECTORS]

ORIENTATIONS = [(0.37,0.52,-0.18,-0.73),(0.37,-0.52,-0.18,-0.73)]
INIT_ORIENTATIONS = []
N = 100
# ~ N = 1
#create session dir
import datetime
session = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
sessionPath = EXPORT_PATH+session
os.makedirs(sessionPath)


from mlp.utils import wholebody_result as wr
# ~ from mlp.utils.util import SE3FromConfig, SE3toVec

from pinocchio import SE3
from pinocchio import Quaternion as QuaternionP

def SE3FromConfig(q):
    # ~ if isinstance(q,types.ListType):
    if (type(q) is list) or (type(q) is tuple):
        q = np.matrix(q).T
    placement = SE3.Identity()
    placement.translation = q[0:3]
    r = QuaternionP(q[6,0],q[3,0],q[4,0],q[5,0])
    placement.rotation = r.matrix()
    return placement


def SE3toVec(M):
    v = np.matrix(np.zeros((12, 1)))
    for j in range(3):
        v[j] = M.translation[j]
        v[j + 3] = M.rotation[j, 0]
        v[j + 6] = M.rotation[j, 1]
        v[j + 9] = M.rotation[j, 2]
    return v

def MotiontoVec(M):
    v = np.matrix(np.zeros((6, 1)))
    for j in range(3):
        v[j] = M.linear[j]
        v[j + 3] = M.angular[j]
    return v


def SE3FromVec(vect):
    if vect.shape[0] != 12 or vect.shape[1] != 1 :
        raise ValueError("SE3FromVect take as input a vector of size 12")
    placement = SE3.Identity()
    placement.translation=vect[0:3]
    rot = placement.rotation
    if len(rot[:,0].shape) == 1 : # depend if eigenpy.switchToNumpyArray() have been called, FIXME : there should be a better way to check this
        rot[:,0]=np.asarray(vect[3:6]).reshape(-1)
        rot[:,1]=np.asarray(vect[6:9]).reshape(-1)
        rot[:,2]=np.asarray(vect[9:12]).reshape(-1)
    else :
        rot[:,0]=vect[3:6]
        rot[:,1]=vect[6:9]
        rot[:,2]=vect[9:12]
    placement.rotation =rot
    return placement



import eigenpy
eigenpy.switchToNumpyMatrix()
def exportPath(pId, directoryPath):
    ln = ps.pathLength(pId)
    nIters = (int)(ln / DT)
    qs = []
    cs = []    
    ts = []    
    effectorTraj = {}
    for effector in EFFECTORS:
        effectorTraj[effector] = []
    for dt in range(nIters):
        t = dt * DT
        ts += [t]
        qs += [ps.configAtParam(pId,t)]; 
        v(qs[-1])
        cs += [robot.getCenterOfMass()]
        for effector in EFFECTORS:
            effectorTraj[effector] += [np.array(SE3toVec(SE3FromConfig(robot.getJointPosition(effector)))).reshape(-1,).tolist()]
    # ~ qs += [ps.configAtParam(pId,ln)]
    # ~ ts += [ln]
    # ~ v(qs[-1])
    # ~ cs += [robot.getCenterOfMass()]
    r = wr.Result(len(robot.getCurrentConfig()),len(robot.getCurrentVelocity()),DT,EFFECTORS,len(qs))
    r.q_t[:] = np.matrix(qs).T
    r.c_t[:] = np.matrix(cs).T
    r.t_t[:] = np.array(ts)    
    for effector in EFFECTORS:
        # ~ print (r.effector_trajectories[effector].shape)
        # ~ print (np.matrix(effectorTraj[effector]))
        r.effector_trajectories[effector][:] = np.matrix(effectorTraj[effector]).T
    r.phases_intervals = None
    fname = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    r.exportNPZ(directoryPath,fname)
    return r

def loadAndPlayPath(filename = '/media/data/memmo/talos_reach0/test/200303_184118'):
    f = pickle.load(open(filename,'rb'),fix_imports=True)
    qs = f['q_t']
    for i in range(qs.shape[1]):
        # ~ return qs[:,i].reshape((-1,)).tolist()
        v(qs[:,i].reshape((-1,)).tolist()[0])
        time.sleep(0.1)

from scipy.spatial import ConvexHull as ch

reachable_space = {}
n_samples = 1000

def randPos(robot, v, ps, effector, orientation):
    ok = False
    while not ok:
        q, ok = genTarget(ps, effector, None, orientation, rand = True)
    v(q)
    return np.array(robot.getJointPosition(effector) [:3])


    
def gen_reachable_workspace():
    for effector, orientation, init_orientation in zip(EFFECTORS, ORIENTATIONS, INIT_ORIENTATIONS):
        # ~ reachable_space[effector] = [randPos(robot, v, ps, effector, orientation) for _ in range(n_samples)]
        reachable_space[effector] = [randPos(robot, v, ps, effector, None) for _ in range(n_samples)]
        hull = ch(reachable_space[effector])
        generators = [hull.points[el] for el in hull.vertices]
        # ~ reachable_space[effector] = [randPos(robot, v, ps, effector, init_orientation) for _ in range(n_samples)]
        # ~ hull = ch(reachable_space[effector])
        # ~ generatorsInit = [hull.points[el] for el in hull.vertices]
        # ~ reachable_space[effector] = [generators, generatorsInit]
        reachable_space[effector] = [generators, generators]
        
        # ~ for i in range(1000):
            # ~ target = gen_rand_conf(effector)
            # ~ if (hull.equations[:,:3].dot(target) + hull.equations[:,-1]).max() > 0.:
                # ~ print ("i", i )
                # ~ print ("err ",  (hull.equations[:,:3].dot(target) + hull.equations[:,-1]).max())

import pickle

def save_reachable_workspace():
    pickle.dump( reachable_space, open( "RW", "wb" ) )
    
def load_reachable_workspace():
    global reachable_space
    reachable_space = pickle.load( open( "RW", "rb" ) )
    
def gen_rand_conf(effector):
    generators = reachable_space[effector][int(np.round(np.random.uniform()))]
    size = len(generators)
    weights = np.random.uniform(size = size)
    weights /= sum(weights)    
    target = sum([ w * el  for w,el in zip(weights, generators)])
    # ~ target = reachable_space[effector][0]
    # ~ print ("generators ", generators)
    # ~ print ("target ", target)
    return target.tolist()
    

def plan_in_reachable(ps, effector, orientation):
    # ~ q1 = genTarget(ps, effector, gen_rand_conf(effector), orientation)
    # ~ q = genTarget(ps, effector, gen_rand_conf(effector), orientation)
    ok = False
    i = 0
    while not ok:
        i +=1
        q1, ok = genTarget(ps, effector, gen_rand_conf(effector), None)
    ok = False
    print ("niters ", i)
    while not ok:
        q, ok = genTarget(ps, effector, gen_rand_conf(effector), None)
    #~ addCons(ps)
    ps.setInitialConfig(q1)
    ps.addGoalConfig(q)
    #~ v(q)
    
scene = 'scene'
v.client.gui.createScene(scene)
v.client.gui.addSceneToWindow(scene,0)
boxId = 0
def createBox(position, avoidCollision = False):
    pos = position[:]
    if len(pos) == 3:
        pos = pos + [0.,0.,0.,1.]
    global boxId
    boxId += 1
    boxsize = 0.05
    bName = scene+"/box"+str(boxId)
    if avoidCollision:
        bName = "box"+str(boxId)
        print ("name ", bName, pos)
        boxsize = 0.1
        ps.client.obstacle.createBox(bName,boxsize,boxsize,boxsize) # x,y,z size
        v.client.gui.addBox(bName,boxsize,boxsize,boxsize,[1.,0.,0.,1.])
        ps.client.obstacle.addObstacle(bName,True,False)
        ps.client.obstacle.moveObstacle(bName, pos)
        v.client.gui.addToGroup(bName,v.sceneName)
    # ~ bName = scene+"/box"+str(boxId)
    else:
        v.client.gui.addBox(bName,boxsize,boxsize,boxsize,[1.,0.,0.,1.])
    v.client.gui.applyConfiguration(bName,pos)
    v.client.gui.refresh()


from hpp.gepetto import PathPlayer
pp = PathPlayer (v)

pps = []


def move_table(pos):
    a = 'planning/table_link'
    q = v.client.gui.getNodeGlobalTransform(a)
    q[:len(pos)]=pos
    v.client.gui.applyConfiguration(a,q)
    v.client.gui.refresh()

lockedJoints = []

def lock_all_but_arm(arm='left'):
    v(init_conf)
    idx = 0
    for jointName in robot.getAllJointNames():
        # ~ if jointName.find('arm_'+arm) < 0 and jointName != "universe" and jointName != "root_joint":
        if (jointName.find('arm_'+arm) < 0 and jointName != "universe" and jointName.find('torso_') < 0) or jointName == "arm_left_5_joint" or jointName == "arm_left_6_joint" or jointName == "arm_left_7_joint" :
        # ~ if jointName.find('arm_'+arm) < 0:
            if robot.getJointConfigSize(jointName) >= 1 and len(init_conf) > idx:
                print ("joint name", jointName)
                qi = robot.getCurrentConfig()[idx]
                # ~ robot.setJointBounds(jointName, [qi,qi])
                global lockedJoints
                ps.createLockedJoint("name"+jointName, jointName, init_conf[idx:idx+robot.getJointConfigSize(jointName)])
                lockedJoints += ["name"+jointName]
        idx += robot.getJointConfigSize(jointName)


import time
def gen_path():
    global pps
    global INIT_ORIENTATIONS
    lock_all_but_arm()
    v(init_conf)    
    INIT_ORIENTATIONS = [robot.getJointPosition(effector)[3:] for effector in EFFECTORS]
    # ~ oldBounds = robot.getJointBounds('torso_1_joint')
    # ~ robot.setJointBounds('torso_1_joint',[oldBounds[0]/2.,oldBounds[1]/2.])
    # ~ oldBounds2 = robot.getJointBounds('torso_2_joint')
    # ~ robot.setJointBounds('torso_2_joint',[oldBounds2[0]/2.,oldBounds2[1]/2.])
    
    # ~ gen_reachable_workspace()
    # ~ save_reachable_workspace()
    # ~ load_reachable_workspace()
    for orientation, effector, generators in zip(ORIENTATIONS, EFFECTORS, START_ZONE_BY_EFFECTORS):
        directoryPath = sessionPath + "/" + effector
        for i in range(N):
            ps.resetGoalConfigs()
            # ~ plan(ps, effector, orientation)
            # ~ plan(ps, effector, None, generators)
            plan(ps, effector, None, None)
            # ~ plan_in_reachable(ps, effector, orientation)
            # ~ plan_in_reachable(ps, effector, orientation)
            ps.solve()
            for i in range(100):
                ps.optimizePath(ps.numberPaths()-1)
            if(ps.pathLength(ps.numberPaths()-1)) < 1.:
            # ~ if True:
                pp(ps.numberPaths()-1)
                # ~ time.sleep(0.5)
                pps += [ps.numberPaths()-1]
                r = exportPath(ps.numberPaths()-1, directoryPath)
            else:
                print ("discarded",ps.pathLength(ps.numberPaths()-1) )
            
# ~ def find_reachable_space():

ps.client.problem.setRandomSeed(np.random.randint(10000))

#create boxes
corner = [-0.25, -0.3, .7425,0.,0.,0.,1.]
offsetTable = [-0.3,-0.1,0.]

#offset table
# ~ move_table(offsetTable)
offsetCorner = [corner[i] + offsetTable[i] for i in range(3)]
offsetCorner = corner[:]
#create boxes
# ~ createBox(offsetCorner,True)
initz = -0.05
initx = -0.2
offsetCorner[2] += initz
offsetCorner[0] += initx
for i in range(2):
    offsetCorner[2] += 0.1
    print ("pos", offsetCorner)
    createBox(offsetCorner,True)
gen_path()



# ~ for path in pps:
    # ~ print ('pid ', path)
    # ~ v(ps.configAtParam(path,0))
    # ~ time.sleep(0.5)
    # ~ time.sleep(0.5)
    
