
id = 2
leg = rhLegId

from disp_bezier import *
from constraint_to_dae import *
s0 = State(fullBody,sId=id)
s2 = State(fullBody,sId=id+1)
s1,success = StateHelper.removeContact(s0,leg)

r(s1.q())
removeAllConstraints(r)
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId,True)
showPath(r,pp,pid)


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



