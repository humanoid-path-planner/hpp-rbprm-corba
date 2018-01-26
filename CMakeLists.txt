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
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-rbprm-corba)
SET(PROJECT_DESCRIPTION "Corba server for reachability based planning")
SET(PROJECT_URL "")

# Set to 1 for profiling
#add_definitions(-DPROFILE)

SET(CUSTOM_HEADER_DIR hpp/corbaserver/rbprm)

SETUP_PROJECT ()

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/rbprm/server.hh
)

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_DOC_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-rbprm")
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-affordance-corba")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
ADD_SUBDIRECTORY(src)

CONFIG_FILES (include/hpp/corbaserver/rbprm/doc.hh)

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  data/package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  data/urdf/hrp2_trunk.urdf
  data/urdf/hrp2_trunk_flexible.urdf
  data/urdf/hrp2_trunk_lowBody_flexible.urdf
  data/urdf/hrp2_trunk_arms_flexible.urdf
  data/urdf/hrp2_rom.urdf
  data/urdf/hrp2_larm_rom.urdf
  data/urdf/hrp2_rarm_rom.urdf
  data/urdf/hrp2_lleg_rom.urdf
  data/urdf/hrp2_rleg_rom.urdf
  data/urdf/hyq/hyq_trunk.urdf
  data/urdf/hyq/hyq_trunk_large.urdf
  data/urdf/hyq/hyq_trunk_large_realist.urdf
  data/urdf/hyq/hyq_rhleg_rom.urdf
  data/urdf/hyq/hyq_rfleg_rom.urdf
  data/urdf/hyq/hyq_lhleg_rom.urdf
  data/urdf/hyq/hyq_lfleg_rom.urdf
  data/urdf/hyq/hyq_rom.urdf
  data/urdf/box_rom.urdf
  data/urdf/box.urdf
  data/urdf/scene.urdf
  data/urdf/scene_wall.urdf
  data/urdf/truck.urdf
  data/urdf/stair_bauzil.urdf
  data/urdf/climb.urdf
  data/urdf/stepladder.urdf
  data/urdf/ground.urdf
  data/urdf/groundcrouch.urdf
  data/urdf/darpa.urdf
  data/urdf/darpa_no_mid.urdf
  data/urdf/darpa_no_ctc_mid.urdf
  data/urdf/car.urdf
  data/urdf/polaris.urdf
  data/urdf/siggraph_asia/chair.urdf
  data/urdf/siggraph_asia/down.urdf
  data/urdf/siggraph_asia/scale.urdf
  data/urdf/siggraph_asia/scale_spidey.urdf
  data/urdf/siggraph_asia/grasp.urdf
  data/urdf/siggraph_asia/plane_hole.urdf
  data/urdf/siggraph_asia/stairs_lower.urdf
  data/urdf/siggraph_asia/wall.urdf
  data/urdf/siggraph_asia/twister.urdf
  data/urdf/siggraph_asia/overview.urdf
  data/urdf/siggraph_asia/mount.urdf
  data/spiderman/urdf/SpidermanLFootSphere.urdf
  data/spiderman/urdf/SpidermanLHandSphere.urdf
  data/spiderman/urdf/SpidermanRFootSphere.urdf
  data/spiderman/urdf/SpidermanRHandSphere.urdf
  data/spiderman/urdf/spiderman_trunk.urdf
  data/spiderman/urdf/spiderman.urdf

  #~ data/urdf/scene2.urdf
  data/urdf/robot_test/robot_test_lleg_rom.urdf
  data/urdf/robot_test/robot_test_rleg_rom.urdf
  data/urdf/robot_test/robot_test_trunk.urdf
	data/urdf/ground_bigStep.urdf
	data/urdf/ground_jump_easy.urdf
	data/urdf/ground_jump_low.urdf
	data/urdf/ground_jump_bar.urdf
	data/urdf/ground_jump_med.urdf
	data/urdf/ground_with_bridge.urdf
	data/urdf/roomWall.urdf
    data/urdf/sideWall.urdf
    data/urdf/sideWall_long.urdf
	data/urdf/downSlope.urdf
	data/urdf/downSlope_easy.urdf
	data/urdf/downSlope_easy20.urdf
	data/urdf/slalom.urdf
	data/urdf/bauzil_stairs.urdf
	data/urdf/flat_step.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  data/srdf/hrp2_trunk.srdf
  data/srdf/hrp2_trunk_flexible.srdf
  data/srdf/hrp2_trunk_lowBody_flexible.srdf
  data/srdf/hrp2_trunk_arms_flexible.srdf
  data/srdf/hrp2_rom.srdf
  data/srdf/hrp2_larm_rom.srdf
  data/srdf/hrp2_rarm_rom.srdf
  data/srdf/hrp2_lleg_rom.srdf
  data/srdf/hrp2_rleg_rom.srdf
  data/srdf/hyq/hyq_trunk.srdf
  data/srdf/hyq/hyq_trunk_large.srdf
  data/srdf/hyq/hyq_trunk_large_realist.srdf
  data/srdf/hyq/hyq_rhleg_rom.srdf
  data/srdf/hyq/hyq_rfleg_rom.srdf
  data/srdf/hyq/hyq_lhleg_rom.srdf
  data/srdf/hyq/hyq_lfleg_rom.srdf
  data/srdf/hyq/hyq_rom.srdf
  data/srdf/box_rom.srdf
  data/srdf/box.srdf
  data/srdf/scene.srdf
  data/srdf/scene_wall.srdf
  data/srdf/truck.srdf
  data/srdf/stair_bauzil.srdf
  data/srdf/climb.srdf
  data/srdf/stepladder.srdf
  data/srdf/ground.srdf
  data/srdf/groundcrouch.srdf
  data/srdf/darpa.srdf
  data/srdf/darpa_no_ctc_mid.srdf
  data/srdf/darpa_no_mid.srdf
  data/srdf/car.srdf
  data/srdf/polaris.srdf
  data/srdf/siggraph_asia/chair.srdf
  data/srdf/siggraph_asia/down.srdf
  data/srdf/siggraph_asia/scale.srdf
  data/srdf/siggraph_asia/scale_spidey.srdf
  data/srdf/siggraph_asia/grasp.srdf
  data/srdf/siggraph_asia/wall.srdf
  data/srdf/siggraph_asia/plane_hole.srdf
  data/srdf/siggraph_asia/stairs_lower.srdf
  data/srdf/siggraph_asia/overview.srdf
  data/srdf/siggraph_asia/twister.srdf
  data/srdf/siggraph_asia/mount.srdf
  data/spiderman/srdf/SpidermanLFootSphere.srdf
  data/spiderman/srdf/SpidermanLHandSphere.srdf
  data/spiderman/srdf/SpidermanRFootSphere.srdf
  data/spiderman/srdf/SpidermanRHandSphere.srdf
  data/spiderman/srdf/spiderman.srdf
  data/spiderman/srdf/spiderman_trunk.srdf

  #~ data/srdf/scene2.srdf
  data/srdf/robot_test/robot_test_lleg_rom.srdf
  data/srdf/robot_test/robot_test_rleg_rom.srdf
  data/srdf/robot_test/robot_test_trunk.srdf
	data/srdf/ground_bigStep.srdf
	data/srdf/ground_jump_easy.srdf
	data/srdf/ground_jump_low.srdf
	data/srdf/ground_jump_bar.srdf
	data/srdf/ground_jump_med.srdf
	data/srdf/ground_with_bridge.srdf
	data/srdf/roomWall.srdf
	data/srdf/sideWall.srdf
	data/srdf/sideWall_long.srdf
	data/srdf/downSlope.srdf
	data/srdf/downSlope_easy.srdf
	data/srdf/downSlope_easy20.srdf
	data/srdf/slalom.srdf
	data/srdf/bauzil_stairs.srdf
	data/srdf/flat_step.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
	data/meshes/darpa.stl
	data/meshes/darpa_no_mid.stl
	data/meshes/darpa_no_ctc_mid.stl
	data/meshes/darpa_no_ctc_mid_collision.stl
	data/meshes/darpareduced.stl
	data/meshes/car.stl
	data/meshes/car_simple.stl
	data/meshes/ground.stl
	data/meshes/ground_table.stl
	data/meshes/hrp2_trunk.stl
	data/meshes/hrp2_trunk_body.stl
	data/meshes/hrp2_trunk_body_low.stl
	data/meshes/hrp2_trunk_torso.stl
	data/meshes/hrp2_trunk_torso_arms.stl
	data/meshes/hrp2_trunk_body_view.dae
	data/meshes/hrp2_trunk_body_low_view.dae
	data/meshes/hrp2_trunk_torso_arms_view.dae
	data/meshes/hrp2_trunk_torso_view.dae
	data/meshes/hrp2_rom.stl
	data/meshes/hrp2_larm_rom.stl
	data/meshes/hrp2_rarm_rom.stl
	data/meshes/hrp2_lleg_rom.stl
	data/meshes/hrp2_rleg_rom.stl
	data/meshes/ground.stl
	data/meshes/box.stl
	data/meshes/box_rom.stl
	data/meshes/robot_box.stl
	data/meshes/chair_simple.stl
	data/meshes/ground.stl
	data/meshes/groundcrouch.stl
	data/meshes/truck.stl
	data/meshes/truck_vision.stl
	data/meshes/stair_bauzil.stl
	data/meshes/stair_bauzil_reduced.stl
	data/meshes/climb.stl
	data/meshes/stepladder.stl
	data/meshes/car.stl
  data/meshes/polaris.stl
  data/meshes/polaris_reduced.stl
	data/meshes/ground_bigStep.stl
	data/meshes/ground_jump_easy.stl
	data/meshes/ground_jump_easyStart.stl
	data/meshes/ground_jump_easyStart2.stl
	data/meshes/ground_jump_easyGoal.stl
	data/meshes/ground_jump_lowGoal.stl
	data/meshes/ground_jump_bar.stl
	data/meshes/ground_jump_med.stl
	data/meshes/ground_with_bridge.stl
	data/meshes/roomWall.stl
	data/meshes/sideWall.stl
	data/meshes/sideWall_long.stl
	data/meshes/downSlope.stl
	data/meshes/downSlope_easy.stl
	data/meshes/downSlope_easy20.stl
	data/meshes/downSlope2.stl
	data/meshes/slalom.stl
	data/meshes/bauzil_stairs.stl
	data/meshes/bauzil_walk_stairs.stl
	data/meshes/flat_step.stl
	data/meshes/polaris.stl
	data/meshes/polaris_reduced.stl
	data/meshes/siggraph_asia/twister.stl
	data/meshes/siggraph_asia/overview.stl
	data/meshes/siggraph_asia/down.stl
	data/meshes/siggraph_asia/scale.stl
    data/meshes/siggraph_asia/chair.stl
    data/meshes/siggraph_asia/scale_spidey.stl
    data/meshes/siggraph_asia/mountain.stl
    data/meshes/siggraph_asia/grasp.stl
    data/meshes/siggraph_asia/plane_hole.stl
    data/meshes/siggraph_asia/wall.stl
    data/meshes/siggraph_asia/stairs_lower.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )


install(FILES
        data/spiderman/meshes/spiderman-LFoot.stl
        data/spiderman/meshes/spiderman-LFootToe.stl
        data/spiderman/meshes/spiderman-LForearm.stl
        data/spiderman/meshes/spiderman-LHand.stl
        data/spiderman/meshes/spiderman-LHumerus.stl
        data/spiderman/meshes/spiderman-LShank.stl
        data/spiderman/meshes/spiderman-LThigh.stl
        data/spiderman/meshes/spiderman-pelvis_box.stl
        data/spiderman/meshes/spiderman-pelvis.stl
        data/spiderman/meshes/spiderman-RFoot.stl
        data/spiderman/meshes/spiderman-RFootToe.stl
        data/spiderman/meshes/spiderman-RForearm.stl
        data/spiderman/meshes/spiderman-RHand.stl
        data/spiderman/meshes/spiderman-RHumerus.stl
        data/spiderman/meshes/spiderman-RShank.stl
        data/spiderman/meshes/spiderman-RThigh.stl
        data/spiderman/meshes/spiderman-skull_box.stl
        data/spiderman/meshes/spiderman-skull.stl
        data/spiderman/meshes/spiderman-thorax_box.stl
        data/spiderman/meshes/spiderman-thorax.stl
        data/spiderman/meshes/lfoot_rom.stl
        data/spiderman/meshes/lhand_rom.stl
        data/spiderman/meshes/rfoot_rom.stl
        data/spiderman/meshes/rhand_rom.stl

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/spiderman
  )

install(FILES
	data/meshes/hyq/hyq_all.stl
	data/meshes/hyq/hyq_all_realist.stl
	data/meshes/hyq/hyq_trunk.stl
	data/meshes/hyq/hyq_trunk_large.stl
	data/meshes/hyq/hyq_trunk_large2.stl
	data/meshes/hyq/hyq_trunk_large3.stl
	data/meshes/hyq/hyq_trunk_large4.stl
	data/meshes/hyq/hyq_rom.stl
	data/meshes/hyq/hyq_rhleg_rom.stl
	data/meshes/hyq/hyq_rfleg_rom.stl
	data/meshes/hyq/hyq_lhleg_rom.stl
	data/meshes/hyq/hyq_lfleg_rom.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/hyq
  )

install(FILES
	data/meshes/robot_test/robot_trunk_view.stl
	data/meshes/robot_test/robot_trunk.stl
	data/meshes/robot_test/Lleg_rom.stl
	data/meshes/robot_test/Rleg_rom.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/robot_test
  )

install(FILES
	data/com_inequalities/LLEG_JOINT0_com_constraints.obj
	data/com_inequalities/RLEG_JOINT0_com_constraints.obj
  data/com_inequalities/LLEG_JOINT0_com_constraints.dae
	data/com_inequalities/RLEG_JOINT0_com_constraints.dae
	data/com_inequalities/LLEG_LINK5_Kinematics.dae
	data/com_inequalities/RLEG_LINK5_Kinematics.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/com_inequalities
  )
  
install(DIRECTORY data/hyq_description
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/../
  )
  
SETUP_PROJECT_FINALIZE()

