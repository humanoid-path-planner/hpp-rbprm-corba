cmd = "ssh -X iwaki 'cd /local/pfernbac/muscod/walk_dynamic/uc-dual/build ; source /local/pfernbac/config.sh ; muscod_release straight_walk_dynamic '"

import subprocess
subprocess.check_output(cmd,shell=True)



