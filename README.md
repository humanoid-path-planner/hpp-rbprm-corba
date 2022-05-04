#  Humanoid Path Planner - RBPRM-CORBA module

[![Pipeline status](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/badges/master/pipeline.svg)](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/commits/master)
[![Coverage report](https://gitlab.laas.fr/humanoid-path-planner/hpp-rbprm-corba/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/humanoid-path-planner/hpp-rbprm-corba/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/humanoid-path-planner/hpp-rbprm-corba/master.svg)](https://results.pre-commit.ci/latest/github/humanoid-path-planner/hpp-rbprm-corba)

Copyright 2015-2020 LAAS-CNRS

Authors: Steve Tonneau, Pierre Fernbach

## Description
hpp-rbprm-corba implements python bindings for hpp-rbprm, and presents a few example files.
Please refer to this [link](https://github.com/humanoid-path-planner/hpp-rbprm) for information on hpp-rbprm.

## Installation from binary package repository

1. Add robotpkg/wip to your apt configuration: http://robotpkg.openrobots.org/robotpkg-wip.html
2. `sudo apt update && sudo apt install robotpkg-pyXX-hpp-rbprm-corba` (replace pyXX with your python version)
3. Then, you will need to export some variables to allow you system to find the executables:

`export PATH=${PATH:+$PATH:}/opt/openrobots/bin:/opt/openrobots/sbin`

`export MANPATH=${MANPATH:+$MANPATH:}/opt/openrobots/man`

`export PYTHONPATH=/opt/openrobots/lib/pythonXX/site-packages:$PYTHONPATH`(replace XX with your python version)

`export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/opt/openrobots/share"`

`export DEVEL_HPP_DIR=/opt/openrobots/`

(you may want to add these to your .bashrc file)

## Installation From source on ubuntu-16.04 64 bit with ros-kinetic

1. Follow this instructions : http://humanoid-path-planner.github.io/hpp-doc/download.html (select 'Devellopement" in the list)
2. Once this installation is complete, run `make rbprm`

## Optional: installing viewer and python bindings for dependencies

If you are planning to use the visualization tools used by the Gepetto team, along with python examples, you may need a few extra steps:

1. Install the gepetto-viewer server

`sudo apt install -qqy robotpkg-pyXX-qt4-gepetto-viewer-corba`

`sudo apt install -qqy robotpkg-pyXX-qt4-hpp-gepetto-viewer`

2. Install the pinocchio bindings

`sudo apt install -qqy robotpkg-pyXX-pinocchio`

3. Install the dae extension for osg

`sudo apt install -qqy robotpkg-osg-dae`

## Documentation

  Open $DEVEL_DIR/install/share/doc/hpp-rbprm-corba/doxygen-html/index.html in a web brower and you
  will have access to the code documentation. If you are using ipython, the documentation of the methods implemented
  is also directly available in a python console.

## Example

  To see the planner in action, one example from our IJRR submission with HyQ is available. Examples with HRP-2 are also provided, though they can only be executed if you have access to HRP-2 model.

  - You can find the scripts in your install directory, in `lib/pythonXX/dist-packages/hpp/corbaserver/rbprm/scenarios/demos` folder.

  - The planning is decomposed in two phases / scripts. First, a root path is computed (`\*_path.py files`). Then, the contacts are generated along the computed path.

  - In order to start a scenario, run:

  `python -im hpp.corbaserver.rbprm demos.hyq_darpa`

   Replace demos.hyq_darpa with the name of any file in the demos or memmo folder to try different scenarios.

  - Once the script have been executed, you can display the results in the viewer (if you installed it):

      - If it was a `\*_path.py` script, you can run:

          - `planner.play_path()` to display the computed guide path

          - `planner.v(planner.q_init)` or `planner.v(planner.q_goal)` to put the robot at the initial / goal position of the planning

      - If it was a a contact generation script, you can run:

          - `cg.display_sequence()` to display the sequence of configurations in contact computed

          - `cg.display_init_config()` or `cg.display_end_config()` to put the robot at the initial / final whole body configuration

          - `cg.v(cg.configs[i])` to display the i-th wholebody configuration of the sequence

          - `cg.play_guide_path()` to display the guide path


## Creating a new scenario script

Start from one of the scripts in the scenarios/demos folder, eg `talos_flatGround.py`.

* All the `\*_path.py` scripts must define a class called `PathPlanner` that inherit from one of the `{robot}_path_planner` classes defined in the scenarios folder.

    * In the run() method, define the environment used and the initial/goal position

    * If further customization is needed, override the required methods from the parent class.

* All the contact generation scripts must define a class called `ContactGenerator` that inherit from one of the `{robot}_contact_generator` classes defined in the scenarios folder.

    * In the constructor of this class, the parent constructor must be called with an instance of the desired `PathPlanner` class, defining the environment and the guide trajectory.

    * This class may override any method from the parent class in order to change the default settings/choices regarding the contact generation
