#pragma once

#include <calib_solvers/TCalibFromLinesParams.h>
#include <QWidget>
#include <mrpt/config/CConfigFile.h>

/**
 * Class for loading and storing parameters related to CCalibFromLines.
 */

namespace Ui
{
class CCalibFromLinesConfig;
}

class CCalibFromLinesConfig : public QWidget
{
	Q_OBJECT

public:
	CCalibFromLinesConfig(QWidget *parent = 0);
	~CCalibFromLinesConfig();

	 /** Set the associated config file and load the values. */
	 void setConfig(mrpt::config::CConfigFile &config_file);

private slots:

	/** Callback to start extracting lines. */
	void extractLinesClicked();

	/** Callback to start forming line correspondences. */
	void matchLinesClicked();

	/** Callback to start the calibration. */
	void calibrateClicked();

	/** Callback to save the estimated calibration to a file. */
	void saveCalibClicked();

private:
	Ui::CCalibFromLinesConfig *m_ui;

	/** The configuration file for the app.*/
	mrpt::config::CConfigFile m_config_file;

	/** The parameters for calibration from lines. */
	TCalibFromLinesParams m_params;
};
