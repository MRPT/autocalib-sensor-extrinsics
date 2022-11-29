SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(SOURCES
	../main.cpp
	CMainWindow.cpp
	observation_tree/CObservationTreeGui.cpp
	viewer/CViewerContainer.cpp
	viewer/TViewerBuffer.h
	config/CCalibFromPlanesConfig.h
	config/CCalibFromPlanesConfig.cpp
	config/CCalibFromLinesConfig.h
	config/CCalibFromLinesConfig.cpp
	interfaces/CTextObserver.h
	interfaces/CPlanesObserver.h
	interfaces/CLinesObserver.h
	interfaces/CCorrespPlanesObserver.h
	interfaces/CCorrespLinesObserver.h
	interfaces/CRtObserver.h
	core_gui/CCalibFromPlanesGui.h
	core_gui/CCalibFromPlanesGui.cpp
	core_gui/CCalibFromLinesGui.h
	core_gui/CCalibFromLinesGui.cpp)

SET(FILES_TO_MOC
	CMainWindow.h
	observation_tree/CObservationTreeGui.h
	viewer/CViewerContainer.h
	config/CCalibFromPlanesConfig.h
	config/CCalibFromLinesConfig.h)

SET(FILES_TO_UIC
	CMainWindow.ui
	viewer/CViewerContainer.ui
	config/CCalibFromPlanesConfig.ui
	config/CCalibFromLinesConfig.ui)

#SET(FILES_TO_RCC
#	resources/)

QT5_WRAP_CPP(MOC_CPPS ${FILES_TO_MOC})
QT5_WRAP_UI(UIC_CPPS ${FILES_TO_UIC})
#QT5_ADD_RESOURCES(RCC_CPPS ${FILES_TO_RCC})

ADD_EXECUTABLE(autocalib_sensor_extrinsics ${SOURCES} ${MOC_CPPS} ${UIC_CPPS} ${RCC_CPPS})

TARGET_LINK_LIBRARIES(autocalib_sensor_extrinsics core Qt5::Widgets ${MRPT_LIBRARIES} ${PCL_LIBRARIES})
