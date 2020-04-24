import argparse
import atexit
import os
import subprocess
import sys
import time
from importlib import import_module

from .utils import ServerManager

# init argument parser
parser = argparse.ArgumentParser(description="Run a hpp-rbprm scenario. \n"
                                 "Take care of starting and closing the viewer and the hpp-rbprm-server.")
parser.add_argument('scenario_name',
                    type=str,
                    help="The name of the scenario script to run. "
                    "If a relative path is given, hpp.corbaserver.rbprm.scenarios is prepended")
parser.add_argument("-n", "--no_viewer", help="Run rbprm without visualization.", action="store_true")

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

with ServerManager('hpp-rbprm-server'):
    # do the same for the viewer, exept if --no-viewer flag is set
    if not args.no_viewer:
        # TODO: we could also use a ServerManager for gepetto-gui.
        subprocess.run(["/usr/bin/killall", "gepetto-gui"])
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
