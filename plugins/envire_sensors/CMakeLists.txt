project(envire_sensors)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "sensors-plugin-envire-mars")
cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

#FIXME the defintions have to the same as in mars/sim/CMakeLists.txt
add_definitions(-DODE11=1 -DdDOUBLE)

pkg_check_modules(PKGCONFIG REQUIRED
			    lib_manager
			    data_broker
			    mars_interfaces
                smurf
                envire_core
                mars_sim
)
include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

add_subdirectory(src)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Rock Logging (next four lines)
add_definitions(-DBASE_LOG_NAMESPACE=${PROJECT_NAME})
pkg_check_modules(BASE REQUIRED "base-lib")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME}
                      ${PKGCONFIG_LIBRARIES}
                      ${BASE_LIBRARIES}
)

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)


set(_INSTALL_DESTINATIONS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION lib
)


# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${PROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

find_package(Rock)
set(ROCK_USE_CXX11 TRUE)
rock_init(envire_motors 0.1)
rock_standard_layout()

find_package(Boost COMPONENTS system filesystem REQUIRED)
target_link_libraries(envire_sensors
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  )
