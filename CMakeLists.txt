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

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/hpp/python.cmake)

SET(PROJECT_NAME hpp-rbprm-corba)
SET(PROJECT_DESCRIPTION "Corba server for reachability based planning")

# Set to 1 for profiling
#add_definitions(-DPROFILE)

SET(CUSTOM_HEADER_DIR hpp/corbaserver/rbprm)

SETUP_HPP_PROJECT ()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/rbprm/server.hh
)

FINDPYTHON()

ADD_DOC_DEPENDENCY("hpp-core >= 4.3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 4.3")
ADD_REQUIRED_DEPENDENCY("hpp-rbprm >= 4.3")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("hpp-affordance-corba")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-pinocchio >= 4.3")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/find-external/CDD")
find_package(CDD REQUIRED)

add_required_dependency("octomap >= 1.8")
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
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

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
ADD_SUBDIRECTORY(src)

CONFIG_FILES (include/hpp/corbaserver/rbprm/doc.hh)

SETUP_HPP_PROJECT_FINALIZE()
