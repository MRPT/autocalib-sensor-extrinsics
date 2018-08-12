INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# List the source files of CORE
SET(SRC
	CObservationTree.h
	CObservationTreeItem.h
	Utils.h
	CPlane.h
	CLine.h
	correspondences.h
	solver.h
	calib_solvers/CExtrinsicCalib.h
	calib_solvers/CCalibFromPlanes.h
	calib_solvers/CCalibFromLines.h
	calib_solvers/TCalibFromPlanesParams.h
	calib_solvers/TCalibFromLinesParams.h
	calib_solvers/TExtrinsicCalibParams.h
	calib_solvers/TSolverResult.h

	CObservationTree.cpp
	CObservationTreeItem.cpp
	correspondences.cpp
	solver.cpp
	calib_solvers/CExtrinsicCalib.cpp
	calib_solvers/CCalibFromPlanes.cpp
	calib_solvers/CCalibFromLines.cpp
)

# CORE library encapsulates the methods and types for the calibration algorithms
ADD_LIBRARY(core ${SRC})
TARGET_LINK_LIBRARIES(core ${MRPT_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES}) #${Boost_SERIALIZATION_LIBRARY}

# Tell CMake that the linker language is C++
SET_TARGET_PROPERTIES(core PROPERTIES LINKER_LANGUAGE CXX)
#TARGET_INCLUDE_DIRECTORIES(core PUBLIC ${PROJECT_SOURCE_DIR}/build)
