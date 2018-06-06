SET(CORE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH "Path to calib_core include folder")
SET(CORE_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})

# List the source files of CORE
SET(SRC
	${CORE_INCLUDE_DIR}/utils/lines.h
	${CORE_INCLUDE_DIR}/utils/planes.h
	${CORE_INCLUDE_DIR}/utils/correspondences.h
	${CORE_INCLUDE_DIR}/utils/solver.h
	${CORE_INCLUDE_DIR}/calib_solvers/CPlaneMatching.h
	${CORE_INCLUDE_DIR}/calib_solvers/CPlaneMatchingParams.h

	${CORE_CPP_DIR}/utils/lines.cpp
	${CORE_CPP_DIR}/utils/planes.cpp
	${CORE_CPP_DIR}/utils/correspondences.cpp
	${CORE_CPP_DIR}/utils/solver.cpp
	${CORE_CPP_DIR}/calib_solvers/CPlaneMatching.cpp
)

# CORE library encapsulates the methods and types for the calibration algorithms
ADD_LIBRARY(core ${SRC})
TARGET_LINK_LIBRARIES(core ${MRPT_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES}) #${Boost_SERIALIZATION_LIBRARY}

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(core PROPERTIES LINKER_LANGUAGE CXX)
#TARGET_INCLUDE_DIRECTORIES(core PUBLIC ${PROJECT_SOURCE_DIR}/build)
