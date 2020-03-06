# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-rbprm-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-rbprm-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  rbprmbuilder
  )

INCLUDE(${HPP_CORBASERVER_CMAKE_PLUGIN})

OMNIIDL_INCLUDE_DIRECTORIES(
  ${HPP_CORBASERVER_DATAROOTDIR}/idl ${CMAKE_SOURCE_DIR}/idl
  )

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/rbprm)

FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/rbprm/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/rbprm
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/rbprm
    ENABLE_DOCSTRING
    STUBS hpp_stubs.rbprm
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    )

  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/rbprm/${IDL}-idl.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/rbprm)
ENDFOREACH()
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/corbaserver/rbprm
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/corbaserver)
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/rbprm
  DESTINATION ${PYTHON_SITELIB}/hpp_stubs)

HPP_ADD_SERVER_PLUGIN(rbprm-corba
  SOURCES
  ${ALL_IDL_CPP_STUBS}
  rbprmbuilder.impl.cc
  rbprmbuilder.impl.hh
  server.cc

  PKG_CONFIG_DEPENDENCIES
  hpp-rbprm
  hpp-corbaserver
  omniORB4
  hpp-util
  hpp-rbprm)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})
ADD_DEPENDENCIES (rbprm-corba generate_idl_cpp)
ADD_DEPENDENCIES (rbprm-corba generate_idl_python)
TARGET_INCLUDE_DIRECTORIES(rbprm-corba PUBLIC ${CDD_INCLUDE_DIRS})

# Stand alone corba server
ADD_EXECUTABLE (hpp-rbprm-server hpp-rbprm-corba.cc)

PKG_CONFIG_USE_DEPENDENCY(hpp-rbprm-server hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(hpp-rbprm-server hpp-pinocchio)

INSTALL (TARGETS hpp-rbprm-server DESTINATION ${CMAKE_INSTALL_BINDIR})

# Install python files : 
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" __init__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" __main__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" client.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" fewstepsplanner.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" rbprmbuilder.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" rbprmfullbody.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" rbprmstate.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" state_alg.py)

# /tools 
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" affordance_centroids.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" com_constraints.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" constraint_to_dae.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" cwc_trajectory_helper.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" cwc_trajectory.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" display_tools.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" generateROMs.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" getSurfaceExtremumPoints.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" __init__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" narrow_convex_hull.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" obj_to_constraints.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" path_to_trajectory.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" plot_analytics.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" sample_random_transition.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" sample_root_config.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" sampleRotation.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" surfaces_from_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" time_out.py)

# /scenarios
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" abstract_contact_generator.py )
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" abstract_path_planner.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" anymal_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" anymal_path_planner.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" hrp2_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" hrp2_path_planner.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" hyq_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" hyq_path_planner.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" __init__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" talos_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" talos_path_planner.py)

# /scenarios/demos
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hrp2_flatGround_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hrp2_flatGround.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hrp2_plateformes_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hrp2_plateformes.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hyq_darpa_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hyq_darpa.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hyq_slalom_debris_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" hyq_slalom_debris.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" __init__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_flatGround_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_flatGround.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_navBauzil_hard_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_navBauzil_hard.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_navBauzil_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_navBauzil.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_plateformes_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_plateformes.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_stairs10_path.py )
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" talos_stairs10.py)

# /scenarios/memmo
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_circle_oriented_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_circle_oriented.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_circle_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_circle.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_platform_random_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" anymal_platform_random.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" __init__.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_bauzil_with_stairs_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_circle_oriented_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_circle_oriented.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_circle_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_circle.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_contact_generator.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_mazeEas_oriented_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_mazeEas_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_moveEffector_flat.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_moveEffector_stairs_m10.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_moveEffector_stairs_m15.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_moveEffector_stairs_p10.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_moveEffector_stairs_p15.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_navBauzil_oriented_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_navBauzil_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_navBauzil.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_platform_random_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_platform_random.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_randomMove_flat.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_randomMove_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_stairs10_random_path.py)
PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" talos_stairs10_random.py)







