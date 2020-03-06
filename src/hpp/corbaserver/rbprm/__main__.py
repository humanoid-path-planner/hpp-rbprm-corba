import subprocess
import time
import atexit
import argparse
from importlib import import_module
import sys
import os


def kill_process(proc):
    proc.kill()


# init argument parser
parser = argparse.ArgumentParser(description="This script must be called with one argumemnt : "
                                 "the name of the script describing the scenario."
                                 "If a relative path is given, "
                                 "hpp.corbaserver.rbprm.scenarios is prepended")
parser.add_argument('scenario_name',
                    type=str,
                    help="The name of the scenario script to run. "
                    "If a relative path is given, hpp.corbaserver.rbprm.scenarios is prepended")
args = parser.parse_args()
# retrieve argument
scenario_name = args.scenario_name
scenario_name = scenario_name.rstrip(".py")  # remove extension if specified
scenario_name.replace("/", ".")  # if the path to the file is given, transform it to a python path
# try to import the given module
try:
    module_scenario = import_module(scenario_name)
except ImportError:
    print("Cannot import " + scenario_name + " try to prepend path")
scenario_name = "hpp.corbaserver.rbprm.scenarios." + scenario_name
try:
    module_scenario = import_module(scenario_name)
except ImportError:
    print("Cannot import " + scenario_name + ". Check if the path is correct")
    sys.exit(1)

# kill already existing instance of the viewer/server
subprocess.run(["killall", "gepetto-gui"])
subprocess.run(["killall", "hpp-rbprm-server"])
# run the viewer/server in background
# stdout and sterr outputs of the child process are redirected to denull (hidden).
# preexec_fn is used to ignore ctrl-c signal send to the main script (otherwise they are forwarded to the child process)
process_viewer = subprocess.Popen("gepetto-gui",
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL,
                                  preexec_fn=os.setpgrp)
process_server = subprocess.Popen("hpp-rbprm-server",
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL,
                                  preexec_fn=os.setpgrp)
# wait a little for the initialization of the server/viewer
time.sleep(3)

# Get ContactGenerator or PathPlanner class from the imported module and run it
if hasattr(module_scenario, 'ContactGenerator'):
    print("# Run contact generation script ...")
    ContactGenerator = getattr(module_scenario, 'ContactGenerator')
    cg = ContactGenerator()
    cg.run()
elif hasattr(module_scenario, 'PathPlanner'):
    print("# Run guide planning script ...")
    PathPlanner = getattr(module_scenario, 'PathPlanner')
    planner = PathPlanner()
    planner.run()
else:
    print("Given script doesn't contain ContactGenerator neither PathPlanner class.")
    sys.exit(1)

# register cleanup methods to kill viewer/server when exiting python interpreter
atexit.register(kill_process, process_server)
atexit.register(kill_process, process_viewer)
