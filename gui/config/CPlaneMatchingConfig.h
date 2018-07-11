#pragma once

#include <core_gui/TPlaneMatchingParams.h>
#include <QWidget>

#include <mrpt/config/CConfigFile.h>

namespace Ui {
class CPlaneMatchingConfig;
}

class CPlaneMatchingConfig : public QWidget
{
	Q_OBJECT

public:
	 CPlaneMatchingConfig(mrpt::config::CConfigFile &config_file, QWidget *parent = 0);
	~CPlaneMatchingConfig();

private slots:
	void startCalib();
	void proceedCalib();
	void saveCalib();
	/** Callback to save the parameters back to the config file. */
	 void saveParamsClicked();

private:
	Ui::CPlaneMatchingConfig *m_ui;
	/** The configuration file for the app.*/
	mrpt::config::CConfigFile m_config_file;
	/** The parameters for calibration by plane matching */
	TPlaneMatchingParams params;
};
