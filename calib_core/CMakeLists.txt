# Include directories
SET(CALIB_CORE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Path to calib_core include folder")
SET(CALIB_CORE_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# List the source files of CALIB_CORE
SET(SRC
	${CALIB_CORE_INCLUDE_DIR}/calib_core.h
	${CALIB_CORE_INCLUDE_DIR}/lines.h
	${CALIB_CORE_INCLUDE_DIR}/planes.h
	${CALIB_CORE_INCLUDE_DIR}/correspondences.h
	${CALIB_CORE_INCLUDE_DIR}/solver.h

	${CALIB_CORE_CPP_DIR}/lines.cpp
	${CALIB_CORE_CPP_DIR}/planes.cpp
	${CALIB_CORE_CPP_DIR}/correspondences.cpp
	${CALIB_CORE_CPP_DIR}/solver.cpp
)

# CALIB_CORE library encapsulates the data types and methods for SphericalRGBD input, output and basic operations
ADD_LIBRARY(calib_core ${SRC})
TARGET_LINK_LIBRARIES(calib_core ${MRPT_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES}) #${Boost_SERIALIZATION_LIBRARY}

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(calib_core PROPERTIES LINKER_LANGUAGE CXX)
