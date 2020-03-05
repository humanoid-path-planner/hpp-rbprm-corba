
id = 5
leg = lLegId

from .disp_bezier import *
from .constraint_to_dae import *
s0 = State(fullBody,sId=id)
s2 = State(fullBody,sId=id+1)
s1,success = StateHelper.removeContact(s0,leg)

r(s1.q())
removeAllConstraints(r)
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId,True)
pp.dt = 0.0001
showPath(r,pp,pid)

displayBezierConstraints(r)
x = [0.781019, 0.243753, 0.788639]

createSphere('s',r)
moveSphere('s',r,x)


fullBody.isReachableFromState(s0.sId,s0.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

r(s0.q())
fullBody.isReachableFromState(s0.sId,s1.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s1.sId,s1.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

r(s2.q())
fullBody.isReachableFromState(s1.sId,s2.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s2.sId,s2.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()



cameraUp = [0.36137786507606506,
 -0.9666459560394287,
 3.9192535877227783,
 -0.7067988514900208,
 -1.1481716688521715e-09,
 1.1491718687750563e-09,
 0.7074145674705505]


cameraSide = [-1.6217347383499146,
 0.8465432524681091,
 2.39125657081604,
 -0.3736702799797058,
 -0.18917302787303925,
 0.4101495146751404,
 0.8101614117622375]

