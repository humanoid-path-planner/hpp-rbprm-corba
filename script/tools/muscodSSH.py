from planning.config import *
import subprocess

if DISPLAY_MUSCOD:
  d=" -X "
else:
  d=" "

cmd = "ssh"+d+"iwaki 'cd "+MUSCOD_DIR+" ; source /local/pfernbac/config.sh ; muscod_release "+DAT_NAME+" ; killall pgxwin_server '"

#cmd = "ssh -X iwaki 'cd /local/pfernbac/muscod/walk/uc-dual/build ; source /local/pfernbac/config.sh ; muscod_release straight_walk_planning ; killall pgxwin_server'"

def call_muscod():
  success = True
  ssh_ok = True
  print "calling muscod for file : "+DAT_NAME
  try :
    subprocess.check_output(cmd,shell=True)
    print "close PGPLOT server to continue ..."
  except subprocess.CalledProcessError as e :
    success = False
    #print "above error 'Error in `muscod_release'' is normal, only red ERROR message are an issue"
    print "Error in call to muscod : "+str(e.returncode)
    if e.returncode == 255:
      ssh_ok = False
  

  print "muscod complete. file generated : "+CONTACT_SEQUENCE_WHOLEBODY_FILE
  return success,ssh_ok

