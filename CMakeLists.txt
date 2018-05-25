CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)

PROJECT(autocalib-sensor-extrinsics)

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

# Set the maximum level of warnings:
SET( EXTRA_WARNINGS OFF CACHE BOOL "Enable extra warnings apart from -Wall")
MARK_AS_ADVANCED(EXTRA_WARNINGS)

# GNU GCC options ================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	# Enable C++17 features in GCC 4.7+
	IF (NOT CMAKE_GCC_VERSION LESS 47)
	        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y") #-std=c++11 | -std=c++1y | -std=c++14 | -std=c++17
	ENDIF (NOT CMAKE_GCC_VERSION LESS 47)
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Shared options between GCC and CLANG:
# ======================================
IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
        	SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -g")
		ADD_DEFINITIONS( -D_DEBUG)
        	ADD_DEFINITIONS( -DDEBUG)
	ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")

        # Whole program optimization
	IF(WHOLE_PROGRAM_OPTIMIZATION)
        	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --combine")
        	SET(EXES_CXX_FLAGS "${EXES_CXX_FLAGS} -fwhole-program --combine")
	ENDIF(WHOLE_PROGRAM_OPTIMIZATION)

        # "-mtune=native" generates code optimized for the detected current processor.
	IF (WIN32 OR "${CMAKE_ARCH}" MATCHES ".*86" OR "${CMAKE_ARCH}" MATCHES "amd64")
		SET(opt_native_def ON)
	ENDIF (WIN32 OR "${CMAKE_ARCH}" MATCHES ".*86" OR "${CMAKE_ARCH}" MATCHES "amd64")

	SET( OPTIMIZE_NATIVE ${opt_native_def} CACHE BOOL "GCC/clang optimizations for current processor (-mtune=native). Requires GCC 4.2+")

	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")  # Not in debug!
        	SET(OPTIMIZE_NATIVE OFF)
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

        # "-ffast-math"
	SET( OPTIMIZE_FFAST-MATH OFF CACHE BOOL "GCC/clang optimization for floating math (-ffast-math).")
	IF(OPTIMIZE_FFAST-MATH)
        	SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -ffast-math")
	ENDIF(OPTIMIZE_FFAST-MATH)

	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CPP_FLAGS}")
ENDIF ()


# Project dependencies
# ======================================
INCLUDE(cmake_modules/script_MRPT.cmake REQUIRED)

# OpenCV is required to process images from regular cameras
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBS_DIR})

# PCL is required to segment 3D planes from depth images
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS}) 

# BOOST is required for the unit tests
FIND_PACKAGE(Boost 1.46.0 REQUIRED system filesystem unit_test_framework serialization)

# Qt5 GUI library
find_package(Qt5Widgets)

# Set the path to this project' sources
ADD_DEFINITIONS(-DPROJECT_SOURCE_PATH="${PROJECT_SOURCE_DIR}")
ADD_SUBDIRECTORY(calib_core)
ADD_SUBDIRECTORY(gui)

#SET( BUILD_EXAMPLES ON CACHE BOOL "Build examples programs to show functions usage")
#ADD_SUBDIRECTORY(examples)

#SET( BUILD_TESTS ON CACHE BOOL "Build tests")
#ADD_SUBDIRECTORY(test)
