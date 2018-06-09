INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# List the source files of CORE
SET(SRC
	utils/lines.h
	utils/planes.h
	utils/correspondences.h
	utils/solver.h
	calib_solvers/CPlaneMatching.h
	calib_solvers/CPlaneMatchingParams.h

	utils/lines.cpp
	utils/planes.cpp
	utils/correspondences.cpp
	utils/solver.cpp
	calib_solvers/CPlaneMatching.cpp
)

# CORE library encapsulates the methods and types for the calibration algorithms
ADD_LIBRARY(core ${SRC})
TARGET_LINK_LIBRARIES(core ${MRPT_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES}) #${Boost_SERIALIZATION_LIBRARY}

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(core PROPERTIES LINKER_LANGUAGE CXX)
#TARGET_INCLUDE_DIRECTORIES(core PUBLIC ${PROJECT_SOURCE_DIR}/build)
