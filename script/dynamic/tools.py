
def addSphere(viewer,color, pos,rotation = None,name=None,radius=0.01):
  gui = viewer.client.gui
  if name==None:
    i=0
    name='sphere_'+str(i)
    while name in gui.getNodeList():
      i=i+1
      name='sphere_'+str(i)
  gui.addSphere(name,radius,color)
  gui.setVisibility(name,"ALWAYS_ON_TOP")
  gui.addToGroup(name,viewer.sceneName)
  if len(pos)==7:
    rotation=pos[3:7]
    pos=pos[0:3]
  if rotation==None:
    rotation=[1,0,0,0]
  else:
    viewer.addLandmark(name,0.1)
  gui.applyConfiguration(name,pos+rotation)
  gui.refresh()

def moveObject(viewer,pos,rotation=[1,0,0,0]):
  viewer.client.gui.applyConfiguration(name,pos+rotation)
  viewer.client.gui.refresh()

def addVector(viewer,rbprmBuilder,color,v,name=None):
  gui = viewer.client.gui
  if name==None:
    i=0
    name='vector_'+str(i)
    while name in gui.getNodeList():
      i=i+1
      name='sphere_'+str(i)
  quat = rbprmBuilder.quaternionFromVector(v[3:6])
  v[3:7] = quat[::]
  gui.addArrow(name,0.02,1,color)
  gui.addToGroup(name,viewer.sceneName)
  gui.setVisibility(name,"ON")
  gui.applyConfiguration(name,v)
  gui.refresh()


