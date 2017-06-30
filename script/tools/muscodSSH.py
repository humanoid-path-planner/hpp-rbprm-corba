from planning.config import *


cmd = "ssh -X iwaki 'cd "+MUSCOD_DIR+" ; source /local/pfernbac/config.sh ; muscod_release "+DAT_NAME+" ; killall pgxwin_server '"

#cmd = "ssh -X iwaki 'cd /local/pfernbac/muscod/walk/uc-dual/build ; source /local/pfernbac/config.sh ; muscod_release straight_walk_planning ; killall pgxwin_server'"

import subprocess
try :
  subprocess.check_output(cmd,shell=True)
  print "close PGPLOT server to continue ..."
except subprocess.CalledProcessError:
  print "above error 'Error in `muscod_release'' is normal, only red ERROR message are an issue"



