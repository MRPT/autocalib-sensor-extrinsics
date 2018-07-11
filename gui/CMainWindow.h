#pragma once

#include <observation_tree/CObservationTreeModel.h>
#include <core_gui/TPlaneMatchingParams.h>
#include <core_gui/CCalibFromPlanesGui.h>
#include <core_gui/CCalibFromLinesGui.h>

#include <QMainWindow>
#include <QSettings>

#include <mrpt/config/CConfigFile.h>

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit CMainWindow(const std::string &config_file_name, QWidget *parent = 0);
	~CMainWindow();
	/** Saves the initial calibration values and grouping values
	 * from the gui back to the config vile.
	 */
	void saveParams();
	CObservationTreeModel* updatedRawlog();
	void runPlaneMatchingCalib(TPlaneMatchingParams params);
	void runLineMatchingCalib();

private slots:
	void algosIndexChanged(int index);
	void loadRawlog();
	void itemClicked(const QModelIndex &);
	void initCalibChanged(double value);

	/** Call back function for the "Sync Observations" button. */
	void syncObservationsClicked();
	/** Call back function for the "Save Parameters" button.
	 * Saves all the app parameters back to the config file.
	 */

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	/** The configuration file for the app. */
	mrpt::config::CConfigFile m_config_file;

	QSettings m_settings;
	/** Location of the most recently loaded rawlog file. */
	QString m_recent_file;

	/** The initial calibration vector. */
	std::array<double,6> m_init_calib;
	/** Stores the originally loaded rawlog. */
	CObservationTreeModel *m_model;
	/** Stores the synchronized (modified) rawlog after re-grouping. */
	CObservationTreeModel *m_sync_model;

	std::shared_ptr<QWidget> m_config_widget;
	/** Object to interact with the core calibration from planes classes. */
	CCalibFromPlanesGui *m_plane_matching;
	/** Object to interact with the core calibration from lines classes. */
	CCalibFromLinesGui *m_line_matching;
	/** Flag to indicate whether a calibration algorithm has started. */
	bool m_calib_started;
};
