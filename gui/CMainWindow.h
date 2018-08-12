#pragma once

#include <observation_tree/CObservationTreeGui.h>
#include <core_gui/CCalibFromPlanesGui.h>
#include <core_gui/CCalibFromLinesGui.h>
#include <config/CCalibFromPlanesConfig.h>
#include <config/CCalibFromLinesConfig.h>

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
	explicit CMainWindow(QWidget *parent = 0);
	~CMainWindow();

	/** Saves the initial calibration params and grouping params
	 * from the gui back to the config file.
	 */
	void saveParams();

	CObservationTreeGui* updatedRawlog();

	/** Triggers the calibration from planes method. */
	void runCalibFromPlanes(TCalibFromPlanesParams *params);

	/** Triggers the calibration from lines method. */
	void runCalibFromLines(TCalibFromLinesParams *params);

	/** Receives the estimated relative transformation from the gui calib classes. */
	void ontReceivingRt(const std::vector<Eigen::Matrix4f> &relative_transformations);

private slots:
	void algosIndexChanged(int index);

	/** Presents a file selection dialog for selecting the rawlog file. */
	void selectRawlogFile();

	/** Presents a file selection dialog for selecting and loading the config file. */
	void loadConfigFile();

	/** Loads the rawlog into memory, and the config file. */
	void loadRawlog();

	/** Changes the displayed sensor pose according to the selected sensor index. */
	void sensorIndexChanged(int index);

	/** Call back function to display any observations item from the listview. */
	void listItemClicked(const QModelIndex &);

	/** Call back function to display any observation item from the grouped observations treeview. */
	void treeItemClicked(const QModelIndex &);

	void initCalibChanged(double value);

	/** Call back function for the "Sync Observations" button. */
	void syncObservationsClicked();

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	/** The configuration file for the app. */
	mrpt::config::CConfigFile m_config_file;

	QSettings m_settings;

	/** Path of the most recently loaded rawlog file. */
	QString m_recent_rlog_path;

	/** Path of the most recently loaded config file. */
	QString m_recent_config_path;

	/** Stores the originally loaded rawlog. */
	CObservationTreeGui *m_model;

	/** Stores the synchronized (modified) rawlog after re-grouping. */
	CObservationTreeGui *m_sync_model;

	/** Pointer to the calibration using planes config widget. */
	std::shared_ptr<CCalibFromPlanesConfig> m_calib_from_planes_config_widget;

	/** Pointer to the calibration using lines config widget. */
	std::shared_ptr<CCalibFromLinesConfig> m_calib_from_lines_config_widget;

	/** Object to interact with the calibration from planes gui class. */
	CCalibFromPlanesGui *m_calib_from_planes_gui;

	/** Object to interact with the calibration from lines gui class. */
	CCalibFromLinesGui *m_calib_from_lines_gui;
};
