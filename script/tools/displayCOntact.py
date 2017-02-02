

q = [[1.9807 , -1.02895 , -0.00518903],[2.66808 , -0.877693 , 0.989208],[2.28841 , -0.944987 , -0.00518903],]

color = r.color.black

for i in range(0,len(q)):
  r.client.gui.addSphere("s"+str(i),0.03,color)
  r.client.gui.applyConfiguration("s"+str(i),q[i]+[1,0,0,0])
  r.client.gui.addToGroup("s"+str(i),r.sceneName)


r.client.gui.refresh()



q2 = [[1.91069 , -1.14894 , -0.00518903],[2.05069 , -1.14895 , -0.00518903],[1.9107 , -0.908949 , -0.00518903],[2.0507 , -0.908952 , -0.00518903],[2.66808 , -0.997691 , 0.91921],[2.66808 , -0.997691 , 1.05921],[2.66808 , -0.757695 , 0.91921],[2.66808 , -0.757695 , 1.05921],[2.21841 , -1.06498 , -0.00518903],[2.35841 , -1.06499 , -0.00518903],[2.21841 , -0.824988 , -0.00518903],[2.35841 , -0.82499 , -0.00518903],]

color = r.color.red

for i in range(0,len(q2)):
  r.client.gui.addSphere("sc"+str(i),0.01,color)
  r.client.gui.applyConfiguration("sc"+str(i),q2[i]+[1,0,0,0])
  r.client.gui.addToGroup("sc"+str(i),r.sceneName)


r.client.gui.refresh()

