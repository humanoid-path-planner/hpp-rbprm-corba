from flatGround_hrp2_interp import * 

print "#    generating MUSCOD problem     #"
print "####################################"
import planning.generate_straight_walk_dynamic_muscod_problem as mp



print "####################################"
print "#    generating MUSCOD problem     #"
print "#               DONE               #"
print "####################################"
print "#         Calling MUSCOD :         #"
print "####################################"

cmd = "ssh -X iwaki 'cd /local/pfernbac/muscod/walk_dynamic/uc-dual/build ; source /local/pfernbac/config.sh ; muscod_release straight_walk_dynamic ; killall pgxwin_server'"

import subprocess
try :
  subprocess.check_output(cmd,shell=True)
  print "close PGPLOT server to continue ..."
except subprocess.CalledProcessError:
  print "above error 'Error in `muscod_release'' is normal, only red ERROR message are an issue"

print "####################################"
print "#         Calling MUSCOD :         #"
print "#               DONE               #"
print "####################################"
print "#         WholeBody script :  :    #"
print "####################################"

import planning.whole_body as wb

print "####################################"
print "#         WholeBody script :  :    #"
print "#               DONE               #"
print "####################################"
