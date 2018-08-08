#pragma once

#include <calib_solvers/TCalibFromPlanesParams.h>
#include <QWidget>

#include <mrpt/config/CConfigFile.h>

/**
 * Class to load and save parameters related to CCalibFromPlanes.
 */

namespace Ui {
class CCalibFromPlanesConfig;
}

class CCalibFromPlanesConfig : public QWidget
{
	Q_OBJECT

public:
	 CCalibFromPlanesConfig(QWidget *parent = 0);
	~CCalibFromPlanesConfig();

	 /** Set the associated config file and load the values. */
	 void setConfig(mrpt::config::CConfigFile &config_file);


private slots:
	void calibrate();
	void extractPlanes();
	void matchPlanes();

	/** Callback to save the estimated calibration to a file. */
	void saveCalibClicked();

	/** Callback to save the parameters back to the config file. */
	 void saveParamsClicked();

private:
	Ui::CCalibFromPlanesConfig *m_ui;

	/** The configuration file for the app.*/
	mrpt::config::CConfigFile m_config_file;

	/** The parameters for calibration from planes. */
	TCalibFromPlanesParams m_params;
};
