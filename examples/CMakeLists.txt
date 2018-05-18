IF(BUILD_EXAMPLES)

PROJECT(examples)

	SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/examples)

	INCLUDE_DIRECTORIES(${CALIB_CORE_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

        # ****************************************************************************************************************** #
        #  Add some examples to test different functions: like reading datasets, extract and display correspondences, etc.   #
        # ****************************************************************************************************************** #
	ADD_EXECUTABLE(example1 example1.cpp)
	SET(DEPENDENCIES    calib_core
		${MRPT_LIBS}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
 		) 
	TARGET_LINK_LIBRARIES(example1 ${DEPENDENCIES})


ENDIF(BUILD_EXAMPLES)
