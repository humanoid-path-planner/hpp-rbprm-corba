
name = 's2'
gui.addSphere(name,0.01,[1,0,0,1])
gui.setVisibility(name,"ALWAYS_ON_TOP")
gui.addToGroup(name,"world")
gui.applyConfiguration(name,p)
gui.refresh()


gui=r.client.gui
name = 's2'
gui.addSphere(name,0.01,[1,0,0,1])
gui.setVisibility(name,"ALWAYS_ON_TOP")
gui.addToGroup(name,r.sceneName)
gui.applyConfiguration(name,p)
gui.refresh()
