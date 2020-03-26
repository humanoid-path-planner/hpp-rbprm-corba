import time


def createSphere(name, r, size=0.01, color=[0, 0, 0, 1]):
    r.client.gui.addSphere(name, size, color)
    r.client.gui.addToGroup(name, r.sceneName)
    r.client.gui.setVisibility(name, 'ALWAYS_ON_TOP')
    r.client.gui.refresh()


def moveSphere(name, r, pos):
    if len(pos) == 3:
        q = pos + [1, 0, 0, 0]
    elif len(pos) == 7:
        q = pos
    else:
        raise ValueError("pos should be of size 3 or 7 ")
    r.client.gui.applyConfiguration(name, q)
    r.client.gui.refresh()


def displayContactSequence(r, configs, pause=1.):
    for i in range(0, len(configs)):
        r(configs[i])
        time.sleep(pause)


def moveObject(viewer, pos, rotation=[1, 0, 0, 0]):
    viewer.client.gui.applyConfiguration(name, pos + rotation)  # noqa TODO
    viewer.client.gui.refresh()


def addVector(viewer, rbprmBuilder, color, v, name=None):
    gui = viewer.client.gui
    if name is None:
        i = 0
        name = 'vector_' + str(i)
        while name in gui.getNodeList():
            i = i + 1
            name = 'sphere_' + str(i)
    quat = rbprmBuilder.quaternionFromVector(v[3:6])
    v[3:7] = quat[::]
    gui.addArrow(name, 0.02, 1, color)
    gui.addToGroup(name, viewer.sceneName)
    gui.setVisibility(name, "ON")
    gui.applyConfiguration(name, v)
    gui.refresh()


def displaySurfaceFromPoints(viewer, p_list, color=[0, 0, 0, 1], name=None):
    if len(p_list) < 2:
        return
    gui = viewer.client.gui
    node_list = gui.getNodeList()
    if name is None and node_list is not None:
        i = 0
        name = 'surface_' + str(i)
        while name in node_list:
            i = i + 1
            name = 'surface_' + str(i)
    gui.addCurve(name, p_list, color)
    gui.addToGroup(name, viewer.sceneName)
    gui.refresh()
