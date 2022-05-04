obsName = "box"
boxSize = [1, 0.1, 1]  # x,y,z
placement = [0, 1, 0, 0, 0, 0.7071, 0.7071]  # x,y,z, position, x,y,z,w, quaternion
# create collision object
ps.client.obstacle.createBox(obsName, boxSize[0], boxSize[1], boxSize[2])  # x,y,z size
ps.client.obstacle.addObstacle(obsName, True, False)
# creat visual object
v.client.gui.addBox(obsName, boxSize[0], boxSize[1], boxSize[2], v.color.black)
v.client.gui.addToGroup(obsName, v.sceneName)

# move obstacle
ps.client.obstacle.moveObstacle(obsName, placement)
# update display :
v.computeObjectPosition()
