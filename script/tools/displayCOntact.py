
# sideWall : 
q2 = [[1.41929 , 0.56273 , 0.3167],[0.633238 , 0.498175 , 0.252145],[1.41049 , -0.498295 , 0.243064],[0.519045 , -0.527337 , 0.272106],]




color = r.color.black
for i in range(0,len(q)):
  r.client.gui.addSphere("s"+str(i),0.03,color)
  r.client.gui.applyConfiguration("s"+str(i),q[i]+[1,0,0,0])
  r.client.gui.setVisibility("s"+str(i),"ALWAYS_ON_TOP")
  r.client.gui.addToGroup("s"+str(i),r.sceneName)

r.client.gui.refresh()


color = r.color.red
for i in range(0,len(q2)):
  r.client.gui.addSphere("sc"+str(i),0.01,color)
  r.client.gui.applyConfiguration("sc"+str(i),q2[i]+[1,0,0,0])
  r.client.gui.addToGroup("sc"+str(i),r.sceneName)
  r.client.gui.setVisibility("sc"+str(i),"ALWAYS_ON_TOP")

r.client.gui.refresh()


color = r.color.blue
name = "vec2"

quat = rbprmBuilder.quaternionFromVector(v[3:6])
v[3:7] = quat[::]
r.client.gui.addArrow(name,0.02,1,color)
r.client.gui.addToGroup(name,r.sceneName)
r.client.gui.setVisibility(name,"ON")
r.client.gui.applyConfiguration(name,v)
r.client.gui.refresh()




#COM : 
name = 'scom'
r.client.gui.addSphere(name,0.01,r.color.red)
r.client.gui.setVisibility(name,"ALWAYS_ON_TOP")
r.client.gui.addToGroup(name,r.sceneName)
r.addLandmark(name,1)

r.client.gui.applyConfiguration(name,fullBody.getCenterOfMass()+[0,0,-1,0])
r.client.gui.refresh()


""""""""""""""""""""""""""""""""""""""""""""
# stairs :
q= [[0.445947 , -0.487313 , 0.15],[0.413544 , -1.19627 , 0.82163],[0.436797 , -0.761877 , 0.15],]

q2 = [[0.325947 , -0.417313 , 0.15],[0.325947 , -0.557313 , 0.15],[0.565947 , -0.417313 , 0.15],[0.565947 , -0.557313 , 0.15],[0.318277 , -1.1339 , 0.773095],[0.318277 , -1.25864 , 0.773084],[0.508811 , -1.13391 , 0.870177],[0.508811 , -1.25865 , 0.870166],[0.316797 , -0.691877 , 0.15],[0.316797 , -0.831877 , 0.15],[0.556797 , -0.691877 , 0.15],[0.556797 , -0.831877 , 0.15],]


""""""""""""""""""""""""""""""""""""""""""""




""""""""""""""""""""""""""""""""""""""""""""
# wall
q = [[1.9807 , -1.02895 , -0.00518903],[2.66808 , -0.877693 , 0.989208],[2.28841 , -0.944987 , -0.00518903],]

q2 = [[1.91069 , -1.14894 , -0.00518903],[2.05069 , -1.14895 , -0.00518903],[1.9107 , -0.908949 , -0.00518903],[2.0507 , -0.908952 , -0.00518903],[2.66808 , -0.997691 , 0.91921],[2.66808 , -0.997691 , 1.05921],[2.66808 , -0.757695 , 0.91921],[2.66808 , -0.757695 , 1.05921],[2.21841 , -1.06498 , -0.00518903],[2.35841 , -1.06499 , -0.00518903],[2.21841 , -0.824988 , -0.00518903],[2.35841 , -0.82499 , -0.00518903],]




""""""""""""""""""""""""""""""""""""""""""""
