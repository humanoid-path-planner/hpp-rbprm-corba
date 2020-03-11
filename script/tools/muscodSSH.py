from config import *
import subprocess
from subprocess import Popen, PIPE, STDOUT
if DISPLAY_MUSCOD:
  d=" -X "
else:
  d=" "

#cmd = "ssh"+d+"iwaki 'cd "+MUSCOD_DIR+" ; source /local/pfernbac/config.sh ; muscod_release "+DAT_NAME+" ; killall pgxwin_server '"

#cmd = "ssh -X iwaki 'cd /local/pfernbac/muscod/walk/uc-dual/build ; source /local/pfernbac/config.sh ; muscod_release straight_walk_planning ; killall pgxwin_server'"

def call_muscod(forceFormulation=False):
  success = True
  ssh_ok = True
  if forceFormulation:
    DAT_NAME = FULL_DAT_NAME
  else :
    DAT_NAME = REDUCED_DAT_NAME
  #cmd = "ssh"+d+"iwaki 'cd "+MUSCOD_DIR+" ; source /local/pfernbac/config.sh ; muscod_release "+DAT_NAME+" ; killall pgxwin_server'"
  cmd = "ssh"+d+"iwaki 'cd "+MUSCOD_DIR+" ; source /local/pfernbac/config.sh ; muscod_release "+DAT_NAME+"' "
  print("calling muscod for file : "+DAT_NAME)
  print("in directory : "+MUSCOD_DIR)
  try :
    #output = subprocess.check_output(cmd,shell=True)
    p = Popen(cmd, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    output = p.stdout.read()    
    print("close PGPLOT server to continue ...")
  except subprocess.CalledProcessError as e :
    success = False
    #print "above error 'Error in `muscod_release'' is normal, only red ERROR message are an issue"
    print("Error in call to muscod : "+str(e.returncode))
    #print "Error output : "+str(e.output)
    if e.returncode == 255:
      ssh_ok = False
    # 134 : normal ? without pgplot
  if output.count("convergence achieved") > 0 :
    success = True
  else :
    success = False
  
  if success :
    print("muscod complete. file generated : "+CONTACT_SEQUENCE_WHOLEBODY_FILE)
  else :
    print("Muscod did not converge.")
  return success,ssh_ok

