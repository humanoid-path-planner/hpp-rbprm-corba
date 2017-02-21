
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
import time

class Player (object):
    def __init__ (self, fullBody,pathPlayer,tp,configs=[],draw=False,optim_effector=False,use_velocity=False,pathId = 0):
        self.viewer = pathPlayer.publisher
        self.tp = tp
        self.pp = pathPlayer
        self.fullBody=fullBody
        self.configs=configs
        self.rLegId = 'rfleg'
        self.rfoot = 'rf_foot_joint'
        self.lLegId = 'lhleg'
        self.lLeg = 'lh_haa_joint'
        self.lfoot = 'lh_foot_joint'
        self.rarmId = 'rhleg'
        self.rHand = 'rh_foot_joint'
        self.larmId = 'lfleg'
        self.lHand = 'lf_foot_joint'
        self.draw=draw
        self.pathId = pathId
        self.use_velocity = use_velocity
        self.optim_effector = optim_effector
        self.limbsCOMConstraints = { self.rLegId : {'file': "hyq/"+self.rLegId+"_com.ineq", 'effector' : self.rfoot},  
						    self.lLegId : {'file': "hyq/"+self.lLegId+"_com.ineq", 'effector' : self.lfoot},  
						    self.rarmId : {'file': "hyq/"+self.rarmId+"_com.ineq", 'effector' : self.rHand},  
						    self.larmId : {'file': "hyq/"+self.larmId+"_com.ineq", 'effector' : self.lHand} }


    def act(self,i, numOptim = 0, use_window = 1, friction = 0.5, optim_effectors = True, time_scale = 1,  verbose = True, draw = False, trackedEffectors = []):
	    return step(self.fullBody, self.configs, i, numOptim, self.pp, self.limbsCOMConstraints, friction, optim_effectors = optim_effectors, time_scale = time_scale, useCOMConstraints = True, use_window = use_window,
	    verbose = verbose, draw = draw, trackedEffectors = trackedEffectors,use_velocity=self.use_velocity, pathId = self.pathId)

    def initConfig(self):
        self.viewer.client.gui.setVisibility("hyq", "ON")
        self.tp.r.client.gui.setVisibility("toto", "OFF")
        self.tp.r.client.gui.setVisibility("hyq_trunk", "OFF")
        if len(self.configs)>1:
            self.viewer(self.configs[0])
        else:
            q_init = self.fullBody.getCurrentConfig()
            q_init[0:7] = self.tp.ps.configAtParam(0,0.01)[0:7] # use this to get the correct orientation
            dir_init = self.tp.ps.configAtParam(0,0.01)[7:10]
            acc_init = self.tp.ps.configAtParam(0,0.01)[10:13]
            # Randomly generating a contact configuration at q_init
            self.fullBody.setCurrentConfig (q_init)
            q_init = self.fullBody.generateContacts(q_init,dir_init,acc_init)
            self.viewer(q_init)

	
    def endConfig(self):
        self.viewer.client.gui.setVisibility("hyq", "ON")
        self.tp.r.client.gui.setVisibility("toto", "OFF")
        self.tp.r.client.gui.setVisibility("hyq_trunk", "OFF")
        if len(self.configs)>1:
            self.viewer(self.configs[len(self.configs)-1])
        else:
            q_goal = self.fullBody.getCurrentConfig()
            q_goal[0:7] = self.tp.ps.configAtParam(0,self.tp.ps.pathLength(0))[0:7] # use this to get the correct orientation
            dir_goal = self.tp.ps.configAtParam(0,0.01)[7:10]
            acc_goal = self.tp.ps.configAtParam(0,0.01)[10:13]
            # Randomly generating a contact configuration at q_init
            self.fullBody.setCurrentConfig (q_goal)
            q_goal = self.fullBody.generateContacts(q_goal,dir_goal,acc_goal)
            self.viewer(q_goal)

    def rootPath(self):
	    self.viewer.client.gui.setVisibility("hyq", "OFF")
	    self.tp.r.client.gui.setVisibility("toto", "OFF")
	    self.viewer.client.gui.setVisibility("hyq", "OFF")
	    self.viewer.client.gui.setVisibility("hyq_trunk", "ON")
	    self.tp.pp(0)
	    self.viewer.client.gui.setVisibility("hyq_trunk", "OFF")
	    self.viewer.client.gui.setVisibility("hyq", "ON")
	    self.tp.cl.problem.selectProblem("default")
	
    def genPlan(self):
	    self.viewer.client.gui.setVisibility("hyq", "ON")
	    self.tp.r.client.gui.setVisibility("toto", "OFF")
	    self.tp.r.client.gui.setVisibility("hyq_trunk", "OFF")
	    start = time.clock() 
	    self.configs = self.fullBody.interpolate(0.12, 10, 10, True)
	    end = time.clock() 
	    print "Contact plan generated in " + str(end-start) + "seconds"
	
    def displayContactPlan(self,pause = 0.5):
	    self.viewer.client.gui.setVisibility("hyq", "ON")
	    self.tp.r.client.gui.setVisibility("toto", "OFF")
	    self.tp.r.client.gui.setVisibility("hyq_trunk", "OFF")
	    for i in range(0,len(self.configs)-1):
		    self.viewer(self.configs[i]);
		    time.sleep(pause)		
		
    def interpolate(self,begin=1,end=0):
        if end==0:
            end = len(self.configs) - 1
        self.viewer.client.gui.setVisibility("hyq", "ON")
        self.tp.r.client.gui.setVisibility("toto", "OFF")
        self.tp.r.client.gui.setVisibility("hyq_trunk", "OFF")
        for i in range(begin,end):
            self.act(i,1,optim_effectors=self.optim_effector,draw=self.draw)
		
    def play(self,frame_rate = 0.01):
	    play_traj(self.fullBody,self.pp,frame_rate)
	

