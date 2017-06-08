
from hpp.corbaserver.rbprm.tools.cwc_trajectory_helper import step, clean,stats, saveAllData, play_traj
import time

class Player(object):
    def __init__ (self, fullBody,pathPlayer,tp,configs=[],draw=False,use_window = 0 ,optim_effector=False,use_velocity=False,pathId = 0):
        self.viewer = pathPlayer.publisher
        self.tp = tp
        self.pp = pathPlayer
        self.fullBody=fullBody
        self.configs=configs
        self.rLegId = 'hrp2_rleg_rom'
        self.lLegId = 'hrp2_lleg_rom'
        #self.rarmId = 'hrp2_rarm_rom'
        self.draw=draw
        self.pathId = pathId
        self.use_velocity = use_velocity
        self.optim_effector = optim_effector
        self.use_window = use_window
        self.limbsCOMConstraints = { self.rLegId : {'file': "hrp2/RL_com.ineq", 'effector' : 'RLEG_JOINT5'},  
				self.lLegId : {'file': "hrp2/LL_com.ineq", 'effector' : 'LLEG_JOINT5'}}
				#self.rarmId : {'file': "hrp2/RA_com.ineq", 'effector' : 'RARM_JOINT5'} }


    def act(self,i, numOptim = 0, use_window = 0, friction = 0.5, optim_effectors = True, time_scale = 1,  verbose = True, draw = False, trackedEffectors = []):
	    return step(self.fullBody, self.configs, i, numOptim, self.pp, self.limbsCOMConstraints, friction, optim_effectors = optim_effectors, time_scale = time_scale, useCOMConstraints = True, use_window = use_window,
	    verbose = verbose, draw = draw, trackedEffectors = trackedEffectors,use_velocity=self.use_velocity, pathId = self.pathId)

    def initConfig(self):
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
      if len(self.configs)>1:
        self.viewer(self.configs[0])
      else:
        q_init = self.fullBody.getCurrentConfig()
        q_init[0:7] = self.tp.ps.configAtParam(self.pathId,0.001)[0:7] # use this to get the correct orientation
        dir_init = self.tp.ps.configAtParam(self.pathId,0.001)[7:10]
        acc_init = self.tp.ps.configAtParam(self.pathId,0.001)[10:13]
        # Randomly generating a contact configuration at q_init
        self.fullBody.setCurrentConfig (q_init)
        q_init = self.fullBody.generateContacts(q_init,dir_init,acc_init)
        self.viewer(q_init)

	
    def endConfig(self):
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
      if len(self.configs)>1:
        self.viewer(self.configs[len(self.configs)-1])
      else:
        q_goal = self.fullBody.getCurrentConfig()
        q_goal[0:7] = self.tp.ps.configAtParam(self.pathId,self.tp.ps.pathLength(self.pathId))[0:7] # use this to get the correct orientation
        dir_goal = self.tp.ps.configAtParam(self.pathId,0.01)[7:10]
        acc_goal = self.tp.ps.configAtParam(self.pathId,0.01)[10:13]
        # Randomly generating a contact configuration at q_init
        self.fullBody.setCurrentConfig (q_goal)
        q_goal = self.fullBody.generateContacts(q_goal,dir_goal,acc_goal)
        self.viewer(q_goal)

    def rootPath(self):
      self.viewer.client.gui.setVisibility("hrp2_14", "OFF")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "ON")
      self.tp.pp(self.pathId)
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
	
    def genPlan(self):
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
      start = time.clock() 
      self.configs = self.fullBody.interpolate(0.12, 10, 10, True)
      end = time.clock() 
      print "Contact plan generated in " + str(end-start) + "seconds"
	
    def displayContactPlan(self,timeScale = 1.):
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
      previousTime = self.fullBody.getTimeAtState(0)
      for i in range(0,len(self.configs)-1):
        self.viewer(self.configs[i]);
        time.sleep(timeScale)
        #nextTime = self.fullBody.getTimeAtState(i+1)
        #time.sleep((nextTime-previousTime)*timeScale)
        #previousTime=nextTime		
		
    def interpolate(self,begin=1,end=0):
      if end==0:
          end = len(self.configs) - 1
      self.viewer.client.gui.setVisibility("hrp2_14", "ON")
      self.tp.r.client.gui.setVisibility("toto", "OFF")
      self.tp.r.client.gui.setVisibility("hrp2_trunk_flexible", "OFF")
      for i in range(begin,end):
          self.viewer(self.configs[i])
          self.act(i,20,use_window=self.use_window,optim_effectors=self.optim_effector,draw=self.draw)

		
    def play(self,frame_rate = 1./30.):
	    play_traj(self.fullBody,self.pp,frame_rate)
	

