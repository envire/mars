# Install script for directory: /home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/CMakeFiles/CMakeRelink.dir/libmars_sim.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mars/sim" TYPE FILE FILES
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/Controller.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/ControllerManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/EntityManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/JointManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/MotorManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/NodeManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/ItemManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/PhysicsMapper.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/SensorManager.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/SimEntity.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/SimJoint.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/SimMotor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/SimNode.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/core/Simulator.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/RotatingRaySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/physics/JointPhysics.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/physics/NodePhysics.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/physics/WorldPhysics.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/CameraSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/IDListConfig.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/Joint6DOFSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointArraySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointAVGTorqueSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointLoadSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointPositionSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointTorqueSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/JointVelocitySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/MotorCurrentSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeAngularVelocitySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeArraySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeCOMSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeContactForceSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeContactSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodePositionSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeRotationSensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/NodeVelocitySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/RaySensor.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/MultiLevelLaserRangeFinder.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/sensors/ScanningSonar.h"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/interfaces/sensors/GridSensorInterface.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mars/sim" TYPE DIRECTORY FILES
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/interfaces/core/"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/interfaces/plugins/"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/interfaces/sensors/"
    "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/src/interfaces/physics/"
    FILES_MATCHING REGEX "/[^/]*\\.h$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/mars_sim.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/dfki.uni-bremen.de/yoo/rockentern/simulation/mars/sim/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
