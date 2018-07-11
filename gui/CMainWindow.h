#pragma once

#include <observation_tree/CObservationTreeModel.h>
#include <core_wrappers/TPlaneMatchingParams.h>
#include <core_wrappers/CCalibFromPlanesWrapper.h>
#include <core_wrappers/CCalibFromLinesWrapper.h>

#include <QMainWindow>
#include <QSettings>

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit CMainWindow(QWidget *parent = 0);
	~CMainWindow();
	CObservationTreeModel* updatedRawlog();
	void runPlaneMatchingCalib(TPlaneMatchingParams params);
	void runLineMatchingCalib();

private slots:
	void algosIndexChanged(int index);
	void loadRawlog();
	void itemClicked(const QModelIndex &);
	void initCalibChanged(double value);

	void syncObservations();

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	QSettings m_settings;
	QString m_recent_file;

	std::array<double,6> m_init_calib;
	CObservationTreeModel *m_model, *m_sync_model;

	std::shared_ptr<QWidget> m_config_widget;
	/** Object to interact with the core calibration from planes classes */
	CCalibFromPlanesWrapper *m_plane_matching;
	/** Object to interact with the core calibration from lines classes */
	CCalibFromLinesWrapper *m_line_matching;
	bool m_calib_started;
};
