SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(SOURCES
	main.cpp
	CMainWindow.cpp
	observation_tree/CObservationTreeModel.cpp
	observation_tree/CObservationTreeItem.cpp
	viewer/CViewerContainer.cpp
	config/CPlaneMatchingConfig.cpp)

SET(FILES_TO_MOC
	CMainWindow.h
	observation_tree/CObservationTreeModel.h
	viewer/CViewerContainer.h
	config/CPlaneMatchingConfig.h)

SET(FILES_TO_UIC
	CMainWindow.ui
	viewer/CViewerContainer.ui
	config/CPlaneMatchingConfig.ui)

#SET(FILES_TO_RCC
#	resources/)

QT5_WRAP_CPP(MOC_CPPS ${FILES_TO_MOC})
QT5_WRAP_UI(UIC_CPPS ${FILES_TO_UIC})
#QT5_ADD_RESOURCES(RCC_CPPS ${FILES_TO_RCC})

ADD_EXECUTABLE(autocalib_sensor_extrinsics ${SOURCES} ${MOC_CPPS} ${UIC_CPPS} ${RCC_CPPS})

TARGET_LINK_LIBRARIES(autocalib_sensor_extrinsics core Qt5::Widgets ${MRPT_LIBRARIES} ${PCL_LIBRARIES})
