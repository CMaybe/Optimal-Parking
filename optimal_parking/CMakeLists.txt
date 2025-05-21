cmake_minimum_required(VERSION 3.27.4)

project(optimal_parking VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(PkgConfig REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)
find_package(OsqpEigen)


add_library(${PROJECT_NAME} SHARED)
target_include_directories(
	${PROJECT_NAME} PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>
)
target_sources(
	${PROJECT_NAME} PRIVATE
		src/system/system_state.cpp
		src/system/system_model.cpp
		src/system/system_input.cpp
		src/trajectory_optimizer.cpp
		src/rrt_star.cpp	
)

target_link_libraries(
	${PROJECT_NAME} PUBLIC
	Eigen3::Eigen
	${YAML_CPP_LIBRARIES}
	OsqpEigen::OsqpEigen
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
	TARGETS optimal_parking
	EXPORT optimal_parking-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
	EXPORT optimal_parking-targets
	FILE optimal_parking-targets.cmake
	NAMESPACE optimal_parking::
	DESTINATION ${INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/optimal_parking-config-version.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/optimal_parking_config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/optimal_parking-config.cmake
	INSTALL_DESTINATION $${INSTALL_CONFIGDIR}
)

install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/optimal_parking-config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/optimal_parking-config-version.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)
