
id = 0
leg = rLegId
s0 = State(fullBody,sId=id)
s2 = State(fullBody,sId=id+1)
s1,success = StateHelper.removeContact(s0,leg)

removeAllConstraints(r)
r(s1.q())
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId)
pp.displayPath(pid,color=r.color.blue)
r.client.gui.setVisibility('path_'+str(pid)+'_root','ALWAYS_ON_TOP')



fullBody.isReachableFromState(s0.sId,s0.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s0.sId,s1.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s1.sId,s1.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s1.sId,s2.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

fullBody.isReachableFromState(s2.sId,s2.sId)
displayOneStepConstraints(r,True)

r.startCapture ("capture/capture","png")
r.stopCapture ()

