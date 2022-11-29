
SET(RV_HAS_MRPT 0)
#SET(RV_HAS_MRPT_BASE 0)

# Leave at the user's choice to disable this library:
OPTION(DISABLE_MRPT "Forces NOT using MRPT, even if it is found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_MRPT)

IF(NOT DISABLE_MRPT)
	# MRPT library:
	# --------------------------------------------
	#find_package(MRPT COMPONENTS base hwdrivers obs slam maps graphslam pbmap QUIET)
	find_package(MRPT COMPONENTS obs serialization rtti maps gui pbmap QUIET)
	IF(MRPT_FOUND)

		SET(RV_HAS_MRPT 1)
		SET(RV_HAS_MRPT_BASE 1)
		add_definitions(-DRV_HAS_MRPT=1)
		#add_definitions(-D_HAS_MRPT_BASE=${RV_HAS_MRPT_BASE})

		INCLUDE_DIRECTORIES(${MRPT_INCLUDE_DIRS})
		link_directories(${MRPT_LIBRARY_DIRS})
		add_definitions(${MRPT_DEFINITIONS})

		IF($ENV{VERBOSE})
			MESSAGE(STATUS "MRPT:")
			MESSAGE(STATUS " Include dirs: ${MRPT_INCLUDE_DIRS}")
			MESSAGE(STATUS " Library dirs: ${MRPT_LIBRARY_DIRS}")
			MESSAGE(STATUS " Definitions : ${MRPT_DEFINITIONS}")
			MESSAGE(STATUS " Libraries   : ${MRPT_LIBRARIES}")
		ENDIF($ENV{VERBOSE})

	ENDIF(MRPT_FOUND)

ENDIF(NOT DISABLE_MRPT)
