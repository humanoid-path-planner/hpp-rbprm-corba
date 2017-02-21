

import time
pp.dt = 1/30.
pp.speed=0.25
r(pp.client.problem.configAtParam(ps.numberPaths ()-1,0)) 
r.startCapture ("capture/capture","png")
time.sleep(2)
pp(ps.numberPaths ()-1)
#r(pp.client.problem.configAtParam(ps.numberPaths ()-1,pp.client.problem.pathLength(ps.numberPaths() - 1)))
time.sleep(2);
r.stopCapture ()


import time
r(configs[1]) 
r.client.gui.startCapture (0,"capture/capture","png")
time.sleep(2)
player.play()
r(configs[len(configs)-2])
time.sleep(2);
r.client.gui.stopCapture(0)

import time
r(configs[0])
tp.r.startCapture ("capture/capture","png")
time.sleep(2)
player.displayContactPlan(1)
tp.r.stopCapture ()

import time
r(configs[6])
tp.r.startCapture ("capture/capture","png")
time.sleep(2)
player.play(1/2.)
tp.r.stopCapture ()




#id = rr.client.gui.getWindowID("window_hpp_")
#rr.client.gui.attachCameraToNode("spiderman/Skull",id)


"""

## avconv (bash) commands
avconv -i capture_0_%d.png -r 30 -vcodec mpeg4 -qscale 1 -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -y downSlope_hrp2_interpolation2.mp4

avconv -i capture_0_%d.png -r 30 -vcodec mpeg4 -qscale 1 -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -pass 2  hyq_darpa.mp4

avconv -i video.mp4 -vcodec mpeg4 -crf 24 prepare_jump.mp4
rm capture*.png
rm video.mp4
"""
