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

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${ALL_IDL_CPP_STUBS}
  rbprmbuilder.impl.cc
  rbprmbuilder.impl.hh
  server.cc
)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} PUBLIC ${CDD_INCLUDE_DIRS})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-rbprm)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-affordance-corba)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-rbprm)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm
  )
INSTALL(
  FILES
	${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/client.py
	${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/rbprmbuilder.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/rbprmfullbody.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/rbprmstate.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/state_alg.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm
  )
INSTALL(
  FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/__init__.py
				${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/generateROMs.py
				${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/com_constraints.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/plot_analytics.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/cwc_trajectory.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/cwc_trajectory_helper.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/path_to_trajectory.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/obj_to_constraints.py
        ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/rbprm/tools/time_out.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/tools
  )
# Stand alone corba server
ADD_EXECUTABLE (hpp-rbprm-server hpp-rbprm-corba.cc)
TARGET_LINK_LIBRARIES (hpp-rbprm-server ${LIBRARY_NAME} hpp-rbprm)
PKG_CONFIG_USE_DEPENDENCY(hpp-rbprm-server hpp-pinocchio)

INSTALL (TARGETS hpp-rbprm-server DESTINATION ${CMAKE_INSTALL_BINDIR})
