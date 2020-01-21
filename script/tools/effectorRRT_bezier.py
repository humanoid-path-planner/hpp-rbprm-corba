
from darpa_hrp2_interStatic import *
from .disp_bezier import *
from hpp.corbaserver.rbprm.tools.path_to_trajectory import *



#fullBody.client.basic.robot.setDimensionExtraConfigSpace(7)
total_paths_ids = []
time_per_paths = []

for stateId in range(len(configs)-2):
    # generate COM traj between two adjacent states
    pidCOM = fullBody.isDynamicallyReachableFromState(stateId,stateId + 1,True)
    showPath(r,pp,pidCOM)
    # call effector-rrt with com traj : pid[1:3]
    paths_ids = fullBody.effectorRRTFromPosBetweenState(stateId,stateId+1,int(pidCOM[1]),int(pidCOM[2]),int(pidCOM[3]),1)
    total_paths_ids += paths_ids[:-1]
    time_per_paths+=[ps.pathLength(int(pidCOM[1]))]
    time_per_paths+=[ps.pathLength(int(pidCOM[2]))]
    time_per_paths+=[ps.pathLength(int(pidCOM[3]))]  






trajectory = gen_trajectory_to_play(fullBody,pp,total_paths_ids,time_per_paths)
play_trajectory(fullBody,pp, trajectory)




"""

fullBody.client.basic.robot.setDimensionExtraConfigSpace(7)
pp.displayPath(8,jointName=rfoot)

"""