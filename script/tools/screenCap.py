

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


r(configs[0])
tp.r.startCapture ("capture/capture","png")
time.sleep(2)
play_trajectory(fullBody,pp, trajectory)
time.sleep(1)
tp.r.stopCapture ()

id = r.client.gui.getWindowID("window_hpp_")
r.client.gui.attachCameraToNode( 'hrp2_14/BODY_0',id)


tp.r.startCapture ("capture/capture","png")
tp.r.stopCapture ()



tp.r.startCapture ("capture/capture","png")
time.sleep(1)
pp(int(total_paths_ids[10]))
time.sleep(1)
tp.r.stopCapture ()



"""

## avconv (bash) commands
avconv -i capture_1_%d.png  -r 25 -vcodec mpeg4 -filter:v "setpts=2.*PTS" -qscale 1 -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -y pg_justin_straightLine.mp4

avconv -i capture_0_%d.png -r 30 -vcodec mpeg4 -qscale 1 -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -pass 2  hyq_darpa.mp4

avconv -i video.mp4 -vcodec mpeg4 -crf 24 prepare_jump.mp4
rm capture*.png
rm video.mp4
"""
