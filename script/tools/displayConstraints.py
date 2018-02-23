
id = 2
leg = rhLegId
s0 = State(fullBody,sId=id)
s2 = State(fullBody,sId=id+1)
s1,success = StateHelper.removeContact(s0,leg)

r(s1.q())
removeAllConstraints(r)
pid = fullBody.isDynamicallyReachableFromState(s0.sId,s2.sId,True)
def showPath(pid):
  if len(pid)==1:
    pp.displayPath(int(pid[0]),color=r.color.blue)
    r.client.gui.setVisibility('path_'+str(int(pid[0]))+'_root','ALWAYS_ON_TOP')
  elif len(pid)==4:
    pp.displayPath(int(pid[1]),color=r.color.green)
    r.client.gui.setVisibility('path_'+str(int(pid[1]))+'_root','ALWAYS_ON_TOP')  
    pp.displayPath(int(pid[2]),color=r.color.blue)
    r.client.gui.setVisibility('path_'+str(int(pid[2]))+'_root','ALWAYS_ON_TOP')  
    pp.displayPath(int(pid[3]),color=r.color.yellow)
    r.client.gui.setVisibility('path_'+str(int(pid[3]))+'_root','ALWAYS_ON_TOP')
  elif len(pid) == 3:
    print "only two phases, not implemented yet."
  else:
    print "no path, test failed."



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



