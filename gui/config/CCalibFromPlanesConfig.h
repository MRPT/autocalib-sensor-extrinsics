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
	 CCalibFromPlanesConfig(mrpt::config::CConfigFile &config_file, QWidget *parent = 0);
	~CCalibFromPlanesConfig();

private slots:
	void calibrate();
	void extractPlanes();
	void matchPlanes();
	void saveCalib();

	/** Callback to save the parameters back to the config file. */
	 void saveParamsClicked();

private:
	Ui::CCalibFromPlanesConfig *m_ui;

	/** The configuration file for the app.*/
	mrpt::config::CConfigFile m_config_file;

	/** The parameters for calibration from planes. */
	TCalibFromPlanesParams params;
};
