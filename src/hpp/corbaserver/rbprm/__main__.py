import subprocess
import time
import atexit
import argparse
from importlib import import_module
import sys
import os


# init argument parser
parser = argparse.ArgumentParser(description="Run a hpp-rbprm scenario. \n"
                                             "Take care of starting and closing the viewer and the hpp-rbprm-server.")
parser.add_argument('scenario_name',
                    type=str,
                    help="The name of the scenario script to run. "
                    "If a relative path is given, hpp.corbaserver.rbprm.scenarios is prepended")
parser.add_argument("-n", "--no_viewer", help="Run rbprm without visualization.",action="store_true")

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

# kill already existing instance of the server
subprocess.run(["killall", "hpp-rbprm-server"])
# run the server in background :
# stdout and stderr outputs of the child process are redirected to devnull (hidden).
# preexec_fn is used to ignore ctrl-c signal send to the main script (otherwise they are forwarded to the child process)
process_server = subprocess.Popen("hpp-rbprm-server",
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL,
                                  preexec_fn=os.setpgrp)
# register cleanup methods to kill server when exiting python interpreter
atexit.register(process_server.kill)

# do the same for the viewer, exept if --no-viewer flag is set
disable_viewer = args.no_viewer
if disable_viewer is None:
    subprocess.run(["killall", "gepetto-gui"])
    process_viewer = subprocess.Popen("gepetto-gui",
                                      stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL,
                                      preexec_fn=os.setpgrp)
    atexit.register(process_viewer.kill)


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


