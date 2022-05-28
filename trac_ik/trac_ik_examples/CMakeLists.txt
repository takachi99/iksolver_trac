cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib
    tf        #ここにtf関係を追加
    tf2
    tf2_ros
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
)

add_executable(ik_tests src/old/ik_tests.cpp)
target_link_libraries(ik_tests
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(trac_ik_jointpub01 src/old/trac_ik_jointpub01.cpp)
target_link_libraries(trac_ik_jointpub01
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)
add_executable(trac_ik_jointpub02 src/old/trac_ik_jointpub02.cpp)
target_link_libraries(trac_ik_jointpub02
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(trac_ik_jointpub025 src/old/trac_ik_jointpub025.cpp)
target_link_libraries(trac_ik_jointpub025
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)
add_executable(trac_ik_jointpub src/trac_ik_jointpub.cpp)
target_link_libraries(trac_ik_jointpub
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(send_frame src/send_frame.cpp)
target_link_libraries(send_frame
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(pos_force_controller src/pos_force_controller.cpp)
target_link_libraries(pos_force_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(array_pub src/old/array_pub.cpp)
target_link_libraries(array_pub
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)


add_executable(send_frame01 src/send_frame01.cpp)
target_link_libraries(send_frame01
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(force_convert src/force_convert.cpp)
target_link_libraries(force_convert
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(external_force_frame_pub src/external_force_frame_pub.cpp)
target_link_libraries(external_force_frame_pub
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)

add_executable(pf_hyb_controller src/pf_hyb_controller.cpp)
target_link_libraries(pf_hyb_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)


add_executable(compliance_ctrl src/compliance_ctrl.cpp)
target_link_libraries(compliance_ctrl
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)



install(TARGETS ik_tests
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
