import time


def createSphere(name,r,size=0.01,color=[0,0,0,1]):
  r.client.gui.addSphere(name,size,color)
  r.client.gui.addToGroup(name,r.sceneName)
  r.client.gui.setVisibility(name,'ALWAYS_ON_TOP')
  r.client.gui.refresh()

def moveSphere(name,r,pos):
  q=pos+[1,0,0,0]
  r.client.gui.applyConfiguration(name,q)
  r.client.gui.refresh()

def displayContactSequence(r,configs,pause=1.):
  for i in range(0,len(configs)):
    r(configs[i])
    time.sleep(pause)	
