

q = [[1.8307 , -1.02895 , -0.00518903],[2.66808 , -0.920141 , 1.00027],[2.13841 , -0.944987 , -0.00518903]]

color = r.color.black

for i in range(0,len(q)):
  r.client.gui.addSphere("s"+str(i),0.03,color)
  r.client.gui.applyConfiguration("s"+str(i),q[i]+[1,0,0,0])
  r.client.gui.addToGroup("s"+str(i),r.sceneName)


r.client.gui.refresh()



q2 = [[1.76069 , -1.14894 , -0.00518903],[1.90069 , -1.14895 , -0.00518903],[1.7607 , -0.908949 , -0.00518903],[1.9007 , -0.908952 , -0.00518903],[2.66808 , -1.04014 , 0.930272],[2.66808 , -1.04014 , 1.07027],[2.66808 , -0.800143 , 0.930272],[2.66808 , -0.800143 , 1.07027],[2.06841 , -1.06498 , -0.00518903],[2.20841 , -1.06499 , -0.00518903],[2.06841 , -0.824988 , -0.00518903],[2.20841 , -0.82499 , -0.00518903],]



color = r.color.red

for i in range(0,len(q2)):
  r.client.gui.addSphere("sc"+str(i),0.01,color)
  r.client.gui.applyConfiguration("sc"+str(i),q2[i]+[1,0,0,0])
  r.client.gui.addToGroup("sc"+str(i),r.sceneName)


r.client.gui.refresh()

