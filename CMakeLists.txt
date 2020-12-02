# Copyright (c) 2012, 2020, CNRS
# Authors: Florent Lamiraux, Guilhem Saurel
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

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-rbprm-corba)
SET(PROJECT_DESCRIPTION "Corba server for reachability based planning")

SET(CUSTOM_HEADER_DIR hpp/corbaserver/rbprm)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)

FINDPYTHON()

ADD_PROJECT_DEPENDENCY("hpp-rbprm" REQUIRED)
ADD_PROJECT_DEPENDENCY("hpp-affordance-corba" REQUIRED)
ADD_PROJECT_DEPENDENCY("hpp-gepetto-viewer")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("octomap >= 1.8")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/CDD")
ADD_PROJECT_DEPENDENCY(CDD REQUIRED)

if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
        include_directories(${OCTOMAP_INCLUDE_DIRS})
        link_directories(${OCTOMAP_LIBRARY_DIRS})
  string(REPLACE "." ";" VERSION_LIST ${OCTOMAP_VERSION})
  list(GET VERSION_LIST 0 OCTOMAP_MAJOR_VERSION)
  list(GET VERSION_LIST 1 OCTOMAP_MINOR_VERSION)
  list(GET VERSION_LIST 2 OCTOMAP_PATCH_VERSION)
  add_definitions (-DOCTOMAP_MAJOR_VERSION=${OCTOMAP_MAJOR_VERSION}
                   -DOCTOMAP_MINOR_VERSION=${OCTOMAP_MINOR_VERSION}
                   -DOCTOMAP_PATCH_VERSION=${OCTOMAP_PATCH_VERSION}  -DHPP_FCL_HAVE_OCTOMAP=1)
        message(STATUS "FCL uses Octomap" ${OCTOMAP_MINOR_VERSION})
else()
        message(STATUS "FCL does not use Octomap")
endif()

SET(${PROJECT_NAME}_HEADERS
  include/${CUSTOM_HEADER_DIR}/server.hh
  )

SET(${PROJECT_NAME}_SOURCES
  src/${PROJECT_NAME}.cc
  )

# Stand alone corba server
ADD_EXECUTABLE(hpp-rbprm-server ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(hpp-rbprm-server PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(hpp-rbprm-server hpp-corbaserver::hpp-corbaserver)
INSTALL(TARGETS hpp-rbprm-server EXPORT ${TARGETS_EXPORT_NAME} DESTINATION bin)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

CONFIG_FILES (include/${CUSTOM_HEADER_DIR}/doc.hh)
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
