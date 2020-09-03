# Copyright (c) 2012, 2020, CNRS
# Author: Florent Lamiraux, Guilhem Saurel
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

  LINK_DEPENDENCIES
  hpp-rbprm::hpp-rbprm
  hpp-corbaserver::hpp-corbaserver

  PKG_CONFIG_DEPENDENCIES
  omniORB4)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})
ADD_DEPENDENCIES (rbprm-corba generate_idl_cpp)
ADD_DEPENDENCIES (rbprm-corba generate_idl_python)
TARGET_INCLUDE_DIRECTORIES(rbprm-corba PUBLIC ${CDD_INCLUDE_DIRS})

SET(${PROJECT_NAME}_PYTHON_FILES
    __init__.py
    __main__.py
    client.py
    fewstepsplanner.py
    rbprmbuilder.py
    rbprmfullbody.py
    rbprmstate.py
    state_alg.py
    utils.py
    )

SET(${PROJECT_NAME}_PYTHON_TOOLS
    affordance_centroids.py
    com_constraints.py
    constants_and_tools.py
    constraint_to_dae.py
    cwc_trajectory_helper.py
    cwc_trajectory.py
    display_tools.py
    generateROMs.py
    getSurfaceExtremumPoints.py
    __init__.py
    narrow_convex_hull.py
    obj_to_constraints.py
    path_to_trajectory.py
    plot_analytics.py
    sample_random_transition.py
    sample_root_config.py
    sampleRotation.py
    surfaces_from_path.py
    time_out.py
    )

SET(${PROJECT_NAME}_PYTHON_SCENARIOS
    abstract_contact_generator.py
    abstract_path_planner.py
    anymal_contact_generator.py
    anymal_path_planner.py
    hrp2_contact_generator.py
    hrp2_path_planner.py
    hyq_contact_generator.py
    hyq_path_planner.py
    __init__.py
    talos_contact_generator.py
    talos_path_planner.py
    solo_contact_generator.py
    solo_path_planner.py
    )

SET(${PROJECT_NAME}_PYTHON_SCENARIOS_DEMOS
    hrp2_flatGround_path.py
    hrp2_flatGround.py
    hrp2_plateformes_path.py
    hrp2_plateformes.py
    hyq_darpa_path.py
    hyq_darpa.py
    hyq_slalom_debris_path.py
    hyq_slalom_debris.py
    __init__.py
    talos_flatGround_path.py
    talos_flatGround.py
    talos_navBauzil_hard_path.py
    talos_navBauzil_hard.py
    talos_navBauzil_path.py
    talos_navBauzil_obstacles_path.py
    talos_navBauzil.py
    talos_navBauzil_obstacles.py
    talos_plateformes_path.py
    talos_plateformes.py
    talos_stairs10_path.py
    talos_stairs10.py
    talos_stairs15_path.py
    talos_stairs15.py
    solo_flatGround_path.py
    solo_flatGround.py
    solo_slalom_path.py
    solo_pallet_path.py
    )

SET(${PROJECT_NAME}_PYTHON_SCENARIOS_MEMMO
    anymal_circle_oriented_path.py
    anymal_circle_oriented.py
    anymal_circle_path.py
    anymal_circle.py
    anymal_contact_generator.py
    anymal_platform_random_path.py
    anymal_platform_random.py
    __init__.py
    talos_bauzil_with_stairs_path.py
    talos_circle_oriented_path.py
    talos_circle_oriented.py
    talos_circle_path.py
    talos_circle.py
    talos_contact_generator.py
    talos_mazeEas_oriented_path.py
    talos_mazeEas_path.py
    talos_moveEffector_flat.py
    talos_moveEffector_stairs_m10.py
    talos_moveEffector_stairs_m15.py
    talos_moveEffector_stairs_p10.py
    talos_moveEffector_stairs_p15.py
    talos_navBauzil_oriented_path.py
    talos_navBauzil_path.py
    talos_navBauzil.py
    talos_platform_random_path.py
    talos_platform_random.py
    talos_randomMove_flat.py
    talos_randomMove_path.py
    talos_stairs10_random_path.py
    talos_stairs10_random.py
    )

# Install python files :
FOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_FILES})

# /tools
FOREACH(file ${${PROJECT_NAME}_PYTHON_TOOLS})
  PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/tools" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_TOOLS})

# /scenarios
FOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS})
  PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS})

# /scenarios/demos
FOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS_DEMOS})
  PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/demos" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS_DEMOS})

# /scenarios/memmo
FOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS_MEMMO})
  PYTHON_INSTALL_ON_SITE("hpp/corbaserver/rbprm/scenarios/memmo" ${file})
ENDFOREACH(file ${${PROJECT_NAME}_PYTHON_SCENARIOS_MEMMO})
