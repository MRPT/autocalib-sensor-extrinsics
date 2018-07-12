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

	/** Saves the initial calibration params and grouping params
	 * from the gui back to the config vile.
	 */
	void saveParams();

	CObservationTreeModel* updatedRawlog();

	/** Triggers the calibration from planes method. */
	void runCalibFromPlanes(TPlaneMatchingParams params);

	/** Triggers the calibration from lines method. */
	void runCalibFromLines();

private slots:
	void algosIndexChanged(int index);
	void loadRawlog();

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

	/** Location of the most recently loaded rawlog file. */
	QString m_recent_file;

	/** The initial calibration vector. */
	std::array<double,6> m_init_calib;

	/** Stores the originally loaded rawlog. */
	CObservationTreeModel *m_model;

	/** Stores the synchronized (modified) rawlog after re-grouping. */
	CObservationTreeModel *m_sync_model;

	std::shared_ptr<QWidget> m_config_widget;

	/** Object to interact with the calibration from planes gui class. */
	CCalibFromPlanesGui *m_calib_from_planes_gui;

	/** Object to interact with the calibration from lines gui class. */
	CCalibFromLinesGui *m_calib_from_lines_gui;

	/** Flag to indicate whether a calibration algorithm has started. */
	bool m_calib_started;
};
