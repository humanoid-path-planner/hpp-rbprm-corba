#  Humanoid Path Planner - RBPRM-CORBA module

[![Pipeline status](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/humanoid-path-planner/hpp-rbprm-corba/master/coverage/)

Copyright 2015 LAAS-CNRS

Author: Steve Tonneau

## Description
hpp-rbprm-corba implements python bindings for hpp-rbprm, and presents a few example files.
Please refer to this [link](https://github.com/humanoid-path-planner/hpp-rbprm) for information on hpp-rbprm.

## Installation from binary package repository

1. Add robotpkg to your apt configuration: http://robotpkg.openrobots.org/debian.html
2. `sudo apt update && sudo apt install robotpkg-hpp-rbprm-corba`
3. Then, you will need to export some variables to allow you system to find the executables:

`export PATH=${PATH:+$PATH:}/opt/openrobots/bin:/opt/openrobots/sbin`

`export MANPATH=${MANPATH:+$MANPATH:}/opt/openrobots/man`

`export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH`

`export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/opt/openrobots/share"`

`export DEVEL_HPP_DIR=/opt/openrobots/`

(you may want to add these to your .bashrc file)

## Installation From source on ubuntu-16.04 64 bit with ros-kinetic

1. Follow this instructions : http://humanoid-path-planner.github.io/hpp-doc/download.html (select 'Devellopement" in the list)
2. Once this installation is complete, run `make rbprm`

## Optional: installing viewer and python bindings for dependencies

If you are planning to use the visualization tools used by the Gepetto team, along with python examples, you may need a few extra steps:

1. Install the gepetto-viwer server

`sudo apt install -qqy robotpkg-py27-qt4-gepetto-viewer-corba`

`sudo apt install -qqy robotpkg-py27-qt4-hpp-gepetto-viewer`

2. Install the pinocchio bindings

`sudo apt install -qqy robotpkg-py27-pinocchio`

3. Install the dae extension for osg

`sudo apt install -qqy robotpkg-osg-dae`

## Documentation

  Open $DEVEL_DIR/install/share/doc/hpp-rbprm-corba/doxygen-html/index.html in a web brower and you
  will have access to the code documentation. If you are using ipython, the documentation of the methods implemented
  is also directly available in a python console.

## Example

  To see the planner in action, one example from our IJRR submission with HyQ is available. Examples with HRP-2 are also provided, though they can only be executed if you have access to HRP-2 model.

  - If you installed the planner form binaries, you need to download the scripts as explained here. Otherwise you can find them directly in script/scenarios/demos folder. For the binary proceudre, create a folder and cd in to it, then type


   `wget https://raw.githubusercontent.com/humanoid-path-planner/hpp-rbprm-corba/master/script/scenarios/demos/darpa_hyq.py`

`wget https://raw.githubusercontent.com/humanoid-path-planner/hpp-rbprm-corba/master/script/scenarios/demos/darpa_hyq_path.py`

`wget https://raw.githubusercontent.com/humanoid-path-planner/hpp-rbprm-corba/devel/script/scenarios/demos/run.sh`

  -  Make the run.sh script executable:`chmod +x run.sh`

  - The planning is decomposed in two phases / scripts. First, a root path is computed (\*_path.py files). Then, the contacts are generated along the computed path (\*_interp.py files). The scripts are located in the folder /scripts/scenarios/demos.

  - To see the different steps of the process run

    ``$ ./run.sh darpa_hyq.py`

  The script include comments explaining the different calls to the library. You can call the different methods a() ... d() to see the different steps of the planning.
