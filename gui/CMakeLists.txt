SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(SOURCES
	../main.cpp
	CMainWindow.cpp
	observation_tree/CObservationTreeModel.cpp
	observation_tree/CObservationTreeItem.cpp
	viewer_container/CViewerContainer.cpp)

SET(FILES_TO_MOC
	CMainWindow.h
	observation_tree/CObservationTreeModel.h
	viewer_container/CViewerContainer.h)

SET(FILES_TO_UIC
	CMainWindow.ui
	viewer_container/CViewerContainer.ui)

#SET(FILES_TO_RCC
#	resources/)

QT5_WRAP_CPP(MOC_CPPS ${FILES_TO_MOC})
QT5_WRAP_UI(UIC_CPPS ${FILES_TO_UIC})
#QT5_ADD_RESOURCES(RCC_CPPS ${FILES_TO_RCC})

ADD_EXECUTABLE(autocalib_sensor_extrinsics ${SOURCES} ${MOC_CPPS} ${UIC_CPPS} ${RCC_CPPS})

TARGET_LINK_LIBRARIES(autocalib_sensor_extrinsics Qt5::Widgets ${MRPT_LIBRARIES})
