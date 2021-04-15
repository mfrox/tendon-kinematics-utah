cmake_minimum_required(VERSION 3.10)
project(tendon-kinematics-export LANGUAGES CXX)

set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING
  "Build type choices: Debug Release RelWithDebInfo"
  FORCE
  )
option(BUILD_TESTING "Build tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PythonInterp 3)
find_package(PythonLibs 3)
find_package(pybind11)

# Qt stuff
if (CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)


option (USE_OPENMP "Use OpenMP" ON)
if (USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")
  else()
    message(STATUS "OpenMP not found")
  endif()
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math")

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(THIRD_PARTY_DIR ${SRC_DIR}/3rdparty)


# util
set(UTIL_HEADERS
  ${SRC_DIR}/util/macros.h
  ${SRC_DIR}/util/openfile_check.h
  ${SRC_DIR}/util/poly.h
  ${SRC_DIR}/util/vector_ops.h
  )

# cliparser
set(CLIPARSER_HEADERS ${SRC_DIR}/cliparser/CliParser.h)
install(FILES ${CLIPARSER_HEADERS} DESTINATION include/cliparser)
list(APPEND LIBTENDON_ALL_HEADERS ${CLIPARSER_HEADERS})

# csv
set(CSV_HEADERS ${SRC_DIR}/csv/Csv.h)
install(FILES ${CSV_HEADERS} DESTINATION include/csv)
list(APPEND LIBTENDON_ALL_HEADERS ${CSV_HEADERS})

# tip-controller
set(TIP_CONTROL_HEADERS
    ${SRC_DIR}/tip-control/tip_control.h
    ${SRC_DIR}/tip-control/Controller.h
    )
install(FILES ${TIP_CONTROL_HEADERS} DESTINATION include/tip-control)
list(APPEND LIBTENDON_ALL_HEADERS ${TIP_CONTROL_HEADERS})

# cpptoml
set(CPPTOML_HEADERS
  ${SRC_DIR}/cpptoml/cpptoml.h
  ${SRC_DIR}/cpptoml/toml_conversions.h
  )
install(FILES ${CPPTOML_HEADERS} DESTINATION include/cpptoml)
list(APPEND LIBTENDON_ALL_HEADERS ${CPPTOML_HEADERS})

# tendon
set(TENDON_HEADERS
  ${SRC_DIR}/tendon/BackboneSpecs.h
  ${SRC_DIR}/tendon/TendonResult.h
  ${SRC_DIR}/tendon/TendonRobot.h
  ${SRC_DIR}/tendon/TendonSpecs.h
  ${SRC_DIR}/tendon/get_r_info.h
  ${SRC_DIR}/tendon/solve_initial_bending.h
  ${SRC_DIR}/tendon/tendon_deriv.h
  )
install(FILES ${TENDON_HEADERS} DESTINATION include/tendon)
list(APPEND LIBTENDON_ALL_HEADERS ${TENDON_HEADERS})

# haptic
set(HAPTIC_HEADERS
  ${SRC_DIR}/haptic/HapticQSubscriber.h
  ${SRC_DIR}/haptic/StreamingTransformer.h
  ${SRC_DIR}/haptic/HapticTransform.h
  )
install(FILES ${HAPTIC_HEADERS} DESTINATION include/haptic)
list(APPEND LIBTENDON_ALL_HEADERS ${HAPTIC_HEADERS})


####################
#    libtendon     #
####################

add_library(tendon SHARED
  ${SRC_DIR}/csv/Csv.cpp
  ${SRC_DIR}/haptic/HapticQSubscriber.cpp
  ${SRC_DIR}/haptic/HapticQSubscriber.cpp
  ${SRC_DIR}/tendon/BackboneSpecs.cpp
  ${SRC_DIR}/tendon/TendonResult.cpp
  ${SRC_DIR}/tendon/TendonRobot.cpp
  ${SRC_DIR}/tendon/TendonSpecs.cpp
  ${SRC_DIR}/tendon/get_r_info.cpp
  ${SRC_DIR}/tendon/solve_initial_bending.cpp
  ${SRC_DIR}/tendon/tendon_deriv.cpp
  ${SRC_DIR}/tip-control/Controller.cpp
  ${LIBTENDON_ALL_HEADERS}
  )
target_include_directories(tendon PUBLIC
  ${SRC_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  )
set_property(TARGET tendon PROPERTY CXX_STANDARD 17)
target_include_directories(tendon PUBLIC ${SRC_DIR})
ament_target_dependencies(tendon
  Eigen3
  rclcpp
  std_msgs
  geometry_msgs
  )
target_link_libraries(tendon
  Boost::filesystem
  Boost::system
  Eigen3::Eigen
  Qt5::Core
  Qt5::Gui
  levmar
  )
target_compile_definitions(tendon PUBLIC CPPTOML_USE_MAP)
install(TARGETS tendon DESTINATION lib/${PROJECT_NAME})


add_subdirectory(${THIRD_PARTY_DIR}/levmar-2.6 3rdparty/levmar-2.6)
add_subdirectory(${THIRD_PARTY_DIR}/libros2qt  3rdparty/libros2qt )

####################
#   applications   #
####################

set(APP_SRC_DIR ${SRC_DIR}/apps)

# tendon shape example
add_executable(tendon_shape_example ${APP_SRC_DIR}/tendon_shape_example.cpp)
set_property(TARGET tendon_shape_example PROPERTY CXX_STANDARD 17)
target_link_libraries(tendon_shape_example tendon)
install(TARGETS tendon_shape_example DESTINATION lib/${PROJECT_NAME})

# estimate tension limits
add_executable(estimate_tension_limits ${APP_SRC_DIR}/estimate_tension_limits.cpp)
set_property(TARGET estimate_tension_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_tension_limits tendon)
install(TARGETS estimate_tension_limits DESTINATION lib/${PROJECT_NAME})

# estimate length limits
add_executable(estimate_length_limits ${APP_SRC_DIR}/estimate_length_limits.cpp)
set_property(TARGET estimate_length_limits PROPERTY CXX_STANDARD 17)
target_link_libraries(estimate_length_limits tendon)
install(TARGETS estimate_length_limits DESTINATION lib/${PROJECT_NAME})

# control app: example of how to use the tip controller
add_executable(control_app ${APP_SRC_DIR}/control_app.cpp)
set_property(TARGET control_app PROPERTY CXX_STANDARD 17)
target_link_libraries(control_app tendon)
install(TARGETS control_app DESTINATION lib/${PROJECT_NAME})

# haptic subscriber example
add_executable(haptic_subscriber_example ${APP_SRC_DIR}/haptic_subscriber_example.cpp)
set_property(TARGET haptic_subscriber_example PROPERTY CXX_STANDARD 17)
target_link_libraries(haptic_subscriber_example tendon libros2qt)
install(TARGETS haptic_subscriber_example DESTINATION lib/${PROJECT_NAME})


#######################
#   python bindings   #
#######################

if (${pybind11_FOUND})
  set(PYTHON_BINDINGS_SRC_DIR ${SRC_DIR}/python-bindings)
  pybind11_add_module(cpptendon
    ${PYTHON_BINDINGS_SRC_DIR}/module_cpptendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_controller.h
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.cpp
    ${PYTHON_BINDINGS_SRC_DIR}/submodule_tendon.h
    )
  target_link_libraries(cpptendon PUBLIC tendon)
  set_property(TARGET cpptendon PROPERTY CXX_STANDARD 17)
  install(TARGETS cpptendon DESTINATION lib/${PROJECT_NAME})
endif()


#############
#   tests   #
#############

if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  set(TEST_FOLDER tests)

  #add_executable(all_tests
  add_executable(all_tests
    ${TEST_FOLDER}/cpptoml/tst_toml_conversions.cpp
    ${TEST_FOLDER}/tendon/tst_BackboneSpecs.cpp
    ${TEST_FOLDER}/tendon/tst_TendonResult.cpp
    ${TEST_FOLDER}/tendon/tst_TendonRobot.cpp
    ${TEST_FOLDER}/tendon/tst_TendonSpecs.cpp
    ${TEST_FOLDER}/tendon/tst_tendon_accuracy.cpp
    ${TEST_FOLDER}/util/tst_poly.cpp
    )
  set_property(TARGET all_tests PROPERTY CXX_STANDARD 17)
  target_include_directories(all_tests PRIVATE ${SRC_DIR})
  target_link_libraries(all_tests
    GTest::GTest
    GTest::Main
    Eigen3::Eigen
    tendon
    )
  gtest_discover_tests(all_tests)
endif()

ament_package()
