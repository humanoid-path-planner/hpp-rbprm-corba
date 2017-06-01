from gen_data_from_rbprm import *

from hpp.corbaserver.rbprm.tools.com_constraints import get_com_constraint
from hpp.gepetto import PathPlayer
from hpp.corbaserver.rbprm.state_alg  import computeIntermediateState

from numpy import matrix, asarray
from numpy.linalg import norm
from spline import bezier


def __curveToWps(curve):
    return asarray(curve.waypoints().transpose()).tolist()


def __Bezier(wps, init_acc = [0.,0.,0.], end_acc = [0.,0.,0.], init_vel = [0.,0.,0.], end_vel = [0.,0.,0.]):
    c = curve_constraints();
    c.init_vel = matrix(init_vel);
    c.end_vel  = matrix(end_vel);
    c.init_acc = matrix(init_acc);
    c.end_acc  = matrix(end_acc);
    matrix_bezier = matrix(wps).transpose()
    curve =  bezier(matrix_bezier, c)
    return __curveToWps(curve), curve
    #~ return __curveToWps(bezier(matrix_bezier))

allpaths = []

def play_all_paths():
    for _, pid in enumerate(allpaths):
        ppl(pid)

def play_all_paths_smooth():
    for i, pid in enumerate(allpaths):
        if i % 2 == 1 :
            ppl(pid)
            
def play_all_paths_qs():
    for i, pid in enumerate(allpaths):
        if i % 2 == 0 :
            ppl(pid)

def test(s1,s2, path = False, use_rand = False, just_one_curve = False, num_optim = 0, effector = False, mu=0.5) :
    q1 = s1.q()
    q2 = s2.q()
    stateid = s1.sId
    stateid1 = s2.sId
    sInt = computeIntermediateState(s1,s2)
    com_1 = s1.getCenterOfMass()
    com_2 = s2.getCenterOfMass()
    createPtBox(viewer.client.gui, 0, com_1, 0.01, [0,1,1,1.])
    createPtBox(viewer.client.gui, 0, com_2, 0.01, [0,1,1,1.])
    data = gen_sequence_data_from_state_objects(s1,s2,sInt,mu = mu)
    c_bounds_1 =   s1.getComConstraint(limbsCOMConstraints)
    c_bounds_mid = sInt.getComConstraint(limbsCOMConstraints)
    c_bounds_2 = s2.getComConstraint(limbsCOMConstraints)
    success, c_mid_1, c_mid_2 = solve_quasi_static(data, c_bounds = [c_bounds_1, c_bounds_2, c_bounds_mid], use_rand = use_rand, mu = mu)
    
    print "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ calling effector", effector
    paths_ids = []
    if path and success:
        #~ fullBody.straightPath([c_mid_1[0].tolist(),c_mid_2[0].tolist()])
        #~ fullBody.straightPath([c_mid_2[0].tolist(),com_2])
        if just_one_curve:
            print "just one curve"
            bezier_0, curve = __Bezier([com_1,c_mid_1[0].tolist(),c_mid_2[0].tolist(),com_2])
            createPtBox(viewer.client.gui, 0, c_mid_1[0].tolist(), 0.01, [0,1,0,1.])
            createPtBox(viewer.client.gui, 0, c_mid_2[0].tolist(), 0.01, [0,1,0,1.])
        
            partions = [0.,0.2,0.8,1.]
            p0 = fullBody.generateCurveTrajParts(bezier_0,partions)
            #testing intermediary configurations 
            print 'paritions:', partions[1], " "
            com_interm1 = curve(partions[1])
            com_interm2 = curve(partions[2])
            print "com_1", com_1
            print "com_1", curve(partions[0])
            print "com_interm1", com_interm1
            print "com_interm2", com_interm2
            print "com_2", com_2
            print "com_2", curve(partions[-1])
            success_proj1 = project_com_colfree(fullBody, stateid  , asarray((com_interm1).transpose()).tolist()[0])
            success_proj2 = project_com_colfree(fullBody, stateid1 , asarray((com_interm2).transpose()).tolist()[0])
            
            #~ if success_proj1:
                #~ q_1 = fullBody.projectToCom(stateid, asarray((com_interm1).transpose()).tolist()[0])
                #~ viewer(q_1)
            
            if not success_proj1:
                print "proj 1 failed"
                return False, c_mid_1, c_mid_2, paths_ids
                
            if not success_proj2:
                print "proj 2 failed"
                return False, c_mid_1, c_mid_2, paths_ids
            
            print "p0", p0
            #~ pp.displayPath(p0+1)
            #~ pp.displayPath(p0+2)
            #~ ppl.displayPath(p0)
            ppl.displayPath(p0+1)
            ppl.displayPath(p0+2)
            ppl.displayPath(p0+3)
            if(effector):
                assert False, "Cant deal with effectors right now"
                #~ paths_ids = [int(el) for el in fullBody.effectorRRT(stateid,p0+1,p0+2,p0+3,num_optim)]
            else:
                paths_ids = [int(el) for el in fullBody.comRRTFromPosBetweenState(stateid,stateid1,p0+1,p0+2,p0+3,num_optim)]
            
        else:
            
            success_proj1 = project_com_colfree(fullBody, stateid  , c_mid_1[0].tolist())
            success_proj2 = project_com_colfree(fullBody, stateid1 , c_mid_2[0].tolist())
            
            if not success_proj1:
                print "proj 1 failed"
                return False, c_mid_1, c_mid_2, paths_ids
                
            if not success_proj2:
                print "proj 2 failed"
                return False, c_mid_1, c_mid_2, paths_ids
            
            print "three curves"
            bezier_0, curve = __Bezier([com_1,c_mid_1[0].tolist()]              , end_acc = c_mid_1[1].tolist() , end_vel = [0.,0.,0.])
            bezier_1, curve = __Bezier([c_mid_1[0].tolist(),c_mid_2[0].tolist()], end_acc = c_mid_2[1].tolist(), init_acc = c_mid_1[1].tolist(), init_vel = [0.,0.,0.], end_vel = [0.,0.,0.])
            bezier_2, curve = __Bezier([c_mid_2[0].tolist(),com_2]              , init_acc = c_mid_2[1].tolist(), init_vel = [0.,0.,0.])
        
            p0 = fullBody.generateCurveTraj(bezier_0)
            print "p0", p0
            fullBody.generateCurveTraj(bezier_1)
            fullBody.generateCurveTraj(bezier_2)
            ppl.displayPath(p0)
            ppl.displayPath(p0+1)
            ppl.displayPath(p0+2)
            paths_ids = [int(el) for el in fullBody.comRRTFromPosBetweenState(stateid,stateid1, p0,p0+1,p0+2,num_optim)]
        #~ paths_ids = []
        global allpaths
        allpaths += paths_ids[:-1]
        #~ allpaths += [paths_ids[-1]]
        #~ pp(paths_ids[-1])
    
        #~ return success, paths_ids, c_mid_1, c_mid_2
    return success, c_mid_1, c_mid_2, paths_ids
#~ data = gen_sequence_data_from_state(fullBody,3,configs)

#~ pp(29),pp(9),pp(17)
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *


def createPtBox(gui, winId, config, res = 0.01, color = [1,1,1,0.3]):
    resolution = res
    global scene
    global b_id
    boxname = scene+"/"+str(b_id)
    b_id += 1
    gui.addBox(boxname,resolution,resolution,resolution, color)
    gui.applyConfiguration(boxname,[config[0],config[1],config[2],1,0,0,0])
    gui.addSceneToWindow(scene,winId)
    gui.refresh()

def test_ineq(stateid, constraints, n_samples = 10, color=[1,1,1,1.]):
    Kin = get_com_constraint(fullBody, stateid, fullBody.getConfigAtState(stateid), constraints, interm = False)
    #~ print "kin ", Kin
    #create box around current com
    fullBody.setCurrentConfig(fullBody.getConfigAtState(stateid))
    com = fullBody.getCenterOfMass()
    bounds_c = flatten([[com[i]-1., com[i]+1.] for i in range(3)]) # arbitrary
    for i in range(n_samples):
        c = array([uniform(bounds_c[2*i], bounds_c[2*i+1]) for i in range(3)])
        print "c: ", c
        if(Kin[0].dot(c)<=Kin[1]).all():
            print "boundaries satisfied"
            createPtBox(viewer.client.gui, 0, c, 0.01, color)
        
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [1,0,0,1])
#~ test_ineq(0,{ lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}, 1000, [0,1,0,1])
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RA_com.ineq", 'effector' : rHand}}, 1000, [0,0,1,1])
#~ test_ineq(0,{ rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'}}, 1000, [0,1,1,1])
#~ test_ineq(0, limbsCOMConstraints, 1000, [0,1,1,1])


def gen(s1, s2, num_optim = 0, ine_curve =True, s = 1., effector = False, mu =0.5, gen_traj = True):
    n_fail = 0;
    #~ viewer(configs[i])
    res =  test(s1, s2, True, False, ine_curve,num_optim, effector, mu)
    if(not res[0]):       
        print "lp failed"
        createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [1,0,0,1.])
        createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [1,0,0,1.])
        found = False
        for j in range(1):
            res = test(s1, s2, True, True, ine_curve, num_optim, effector, mu)               
            createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [0,1,0,1.])
            createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [0,1,0,1.])
            if res[0]:
                break
        if not res[0]:
            n_fail += 1
    print "n_fail ", n_fail
    if(gen_traj):
        a = gen_trajectory_to_play(fullBody, ppl, allpaths, flatten([[s*0.2, s* 0.6, s* 0.2] for _ in range(len(allpaths) / 3)]))
        #~ a = gen_trajectory_to_play(fullBody, ppl, allpaths, flatten([[s] for _ in range(len(allpaths) )]))
        return a


def gen_several_states(start = 0, len_con = 1, num_optim = 0, ine_curve =True, s = 1., effector = False, mu =0.5, init_vel = [0.,0.,0.], init_acc = [0.,0.,0.]):
    com_1 = __get_com(fullBody, fullBody.getConfigAtState(start))
    com_2 = __get_com(fullBody, fullBody.getConfigAtState(start+len_con))
    com_vel = init_vel[:]
    com_acc = init_acc[:]
    print "going from, to ", com_1, "->", com_2
    print "going from, to ", start, "->", start + len_con
    allpoints = [com_1]  
    all_partitions = [] 
    n_fail = 0;
    for i in range (start, start+len_con):
        #~ viewer(configs[i])
        res =  test(i, False, False, ine_curve,num_optim, effector, mu)
        if(not res[0]):       
            print "lp failed"
            createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [1,0,0,1.])
            createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [1,0,0,1.])
            found = False
            for j in range(10):
                res = test(i, False, True, ine_curve, num_optim, effector, mu)               
                createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [0,1,0,1.])
                createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [0,1,0,1.])
                if res[0]:
                    allpoints+=[res[1][0],res[2][0]] 
                    step = (1./ len_con)
                    idx = step * (i - start)
                    all_partitions +=  [idx +0.2*step,idx+0.8*step,idx+step]
                    break
            if not res[0]:
                n_fail += 1
        else:      
            allpoints+=[res[1][0],res[2][0]] 
            step = (1./ len_con)
            idx = step * (i - start)
            all_partitions +=  [idx +0.2*step,idx+0.8*step,idx+step]
    all_partitions =  [0.] + all_partitions
    print "n_fail ", n_fail
    print "generating super curve"
    print all_partitions
    allpoints+=[com_2] 
    bezier_0, curve = __Bezier(allpoints)    
    
    
    com_vel = curve.derivate(0.5,1)
    com_acc = curve.derivate(0.5,2)
    com_vel = flatten(asarray(com_vel).transpose().tolist())
    com_acc = flatten(asarray(com_acc).transpose().tolist())    
    print "at", 0.5
    print "com_vel", com_vel
    print "com_acc", com_acc
    
    
    
    com_vel = curve.derivate(all_partitions[-1],1)
    com_acc = curve.derivate(all_partitions[-1],2)
    com_vel = flatten(asarray(com_vel).transpose().tolist())
    com_acc = flatten(asarray(com_acc).transpose().tolist())
    
    print "at", all_partitions[-1]
    print "com_vel", com_vel
    print "com_acc", com_acc
    
    p0 = fullBody.generateCurveTrajParts(bezier_0,all_partitions) + 1
    ppl.displayPath(p0-1)
    # now we need to project all states to the new com positions
    print "WTF ", len(all_partitions)
    for k in range(3, len(all_partitions),3):
        print "k ", k
        print all_partitions[k]
        new_com = flatten(asarray(curve(all_partitions[k]).transpose()).tolist())
        print "curve end ", curve(1.)
        ok = False
        #~ try:
        sid = start+k/3
        print "for state", sid
        print "before project to new com ", new_com
        print "before previous com", __get_com(fullBody, fullBody.getConfigAtState(sid))
        ok = fullBody.projectStateToCOM(sid, new_com)
        print "projection", ok
        if ok:
            q1 = fullBody.getConfigAtState(sid)
            ok = fullBody.isConfigValid(q1)[0]
            print "is config valud", ok
        #~ except:
            #~ print "hpperr"
            #~ break
        if not ok:
            print "faield to project"
            return
    j = 0;
    print "WTF2"
    for i in range(p0,p0+len_con*3,3):
        print "paths ids", i, " ", i+1, " ", i+3
        print "state ", start + j
        paths_ids = [int(el) for el in fullBody.comRRTFromPos(start+j,i,i+1,i+2,num_optim)]    
        j += 1
        global allpaths
        allpaths += paths_ids[:-1]
    #~ p0 = fullBody.generateCurveTrajParts(bezier_0,partions)
        
    a = gen_trajectory_to_play(fullBody, ppl, allpaths, flatten([[s*0.2, s* 0.6, s* 0.2] for _ in range(len(allpaths) / 3)]))
    return a, com_vel, com_acc
    
def gen_several_states_partial(start = 0, len_con = 1, num_optim = 0, ine_curve =True, s = 1., effector = False, mu =0.5, init_vel = [0.,0.,0.], init_acc = [0.,0.,0.], path = False):
    com_1 = __get_com(fullBody, fullBody.getConfigAtState(start))
    com_2 = __get_com(fullBody, fullBody.getConfigAtState(start+len_con))
    com_vel = init_vel[:]
    com_acc = init_acc[:]
    print "going from, to ", com_1, "->", com_2
    #~ print "going from, to ", start, "->", start + len_con
    allpoints = [com_1]  
    all_partitions = [] 
    n_fail = 0;
    for i in range (start, start+len_con):
        #~ viewer(configs[i])
        res =  test(i, False, False, ine_curve,num_optim, effector, mu)
        if(not res[0]):       
            print "lp failed"
            createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [1,0,0,1.])
            createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [1,0,0,1.])
            found = False
            for j in range(10):
                res = test(i, False, True, ine_curve, num_optim, effector, mu)               
                createPtBox(viewer.client.gui, 0, res[1][0], 0.01, [0,1,0,1.])
                createPtBox(viewer.client.gui, 0, res[2][0], 0.01, [0,1,0,1.])
                if res[0]:
                    allpoints+=[res[1][0],res[2][0]] 
                    step = (1./ len_con)
                    idx = step * (i - start)
                    all_partitions +=  [idx +0.2*step,idx+0.8*step,idx+step]
                    break
            if not res[0]:
                n_fail += 1
        else:      
            allpoints+=[res[1][0],res[2][0]] 
            step = (1./ len_con)
            idx = step * (i - start)
            all_partitions +=  [idx +0.2*step,idx+0.8*step,idx+step]
    print "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[", all_partitions
    allpoints+=[com_2] 
    bezier_0, curve = __Bezier(allpoints, init_acc = com_acc, init_vel = com_vel)
    all_partitions =  [0.] + all_partitions[:-3]
    com_vel = curve.derivate(all_partitions[-1],1)
    com_acc = curve.derivate(all_partitions[-1],2)
    com_vel = flatten(asarray(com_vel).transpose().tolist())
    com_acc = flatten(asarray(com_acc).transpose().tolist())
    p0 = fullBody.generateCurveTrajParts(bezier_0,all_partitions) + 1
    print all_partitions
    #~ ppl.displayPath(p0-1)
    ppl.displayPath(p0)
    ppl.displayPath(p0+1)
    ppl.displayPath(p0+2)
    #~ ppl.displayPath(p0)
    # now we need to project all states to the new com positions
    for k in range(3, len(all_partitions),3):
        print "k ", k
        print all_partitions[k]
        new_com = flatten(asarray(curve(all_partitions[k]).transpose()).tolist())
        ok = False
        #~ try:
        sid = start+k/3
        print "for state", sid
        print "before project to new com ", new_com
        print "before previous com", __get_com(fullBody, fullBody.getConfigAtState(sid))
        #~ new_com[0]+=0.02
        ok = fullBody.projectStateToCOM(sid, new_com)
        #~ print "projection", ok
        if ok:
            q1 = fullBody.getConfigAtState(sid)
            ok = fullBody.isConfigValid(q1)[0]
            #~ print "is config valud", ok
        #~ except:
            #~ print "hpperr"
            #~ break
        if not ok:
            print "faield to project"
            return [], com_vel, com_acc
    j = 0;
    #~ print "WTF2"
    if path:
        for i in range(p0,p0+len_con*3-3,3):
            try:
                #~ print "FOR STATE ", start+j 
                #~ print "USING PATHS", i 
                paths_ids = [int(el) for el in fullBody.comRRTFromPos(start+j,i,i+1,i+2,num_optim)]    
                #~ paths_ids = [int(el) for el in fullBody.effectorRRT(start+j,i,i+1,i+2,num_optim)]    
            except:
                print "COULD NOT SOLVE COMRRT"
                return [], com_vel, com_acc
            j += 1
            global allpaths
            allpaths += paths_ids[:-1]
    #~ p0 = fullBody.generateCurveTrajParts(bezier_0,partions)
        
    #~ a = gen_trajectory_to_play(fullBody, ppl, allpaths, flatten([[s*0.2, s* 0.6, s* 0.2] for _ in range(len(allpaths) / 3)]))
    a = [] #TODO
    return a, com_vel, com_acc
    
    
    
    
viewer = None
tp = None
ppl = None
fullBody = None
b_id = 0
scene = "bos"

def clean_path():
    global allpaths
    allpaths = []


def init_bezier_traj(robot, r, pplayer, qs, comConstraints):
    global viewer
    global tp
    global ppl
    global fullBody
    global viewer
    global configs
    configs = qs
    viewer = r
    ppl = pplayer
    fullBody = robot
    viewer.client.gui.createScene(scene)
    global limbsCOMConstraints
    limbsCOMConstraints = comConstraints
    
com_vel = [0.,0.,0.]
com_acc = [0.,0.,0.]

vels = []
accs = []

path = []
a_s = []

        
def go0(states, num_optim = 0, mu = 0.6, s =None):
    global com_vel
    global com_acc
    global vels
    global accs
    global path    
    sc = s
    for i, el in enumerate(states[:-1]):
        if s == None:
            sc = max(norm(array(states[i+1].q()) - array(el.q())), 1.) * 0.5
        path = gen(el,states[i+1],mu=mu,num_optim=num_optim, s=sc)
    return path

    
def reset():
    global com_vel
    global com_acc
    global vels
    global accs
    global a_s
    global path
    com_vel = [0.,0.,0.]
    com_acc = [0.,0.,0.]
    clean_path();
    vels = []
    accs = []
    path = []
    a_s = []
    for i, config in enumerate(configs):
		fullBody.setConfigAtState(i,config)
