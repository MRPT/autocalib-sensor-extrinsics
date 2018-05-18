IF(BUILD_TESTS)

PROJECT(test)

	SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/test)

	enable_testing()

	FIND_PACKAGE(Boost REQUIRED system filesystem unit_test_framework)
	INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )
	ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

	INCLUDE_DIRECTORIES(${CALIB_CORE_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} )
	#INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

	SET(DEPENDENCIES calib_core
		${MRPT_LIBS}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}
		${Boost_FILESYSTEM_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
		${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
		)

        # **************************************************************************************************** #
        #                        A small set of code for quick testing of general stuff                        #
        # **************************************************************************************************** #
	ADD_EXECUTABLE(test1 test1.cpp)
	TARGET_LINK_LIBRARIES(test1 ${DEPENDENCIES})

ENDIF(BUILD_TESTS)
