#pragma once

//#include <calib_solvers/CPlaneMatching.h>

#include <QWidget>

namespace Ui {
class CPlaneConfig;
}

class CPlaneConfig : public QWidget
{
	Q_OBJECT

public:
	explicit CPlaneConfig(QWidget *parent = 0);
	~CPlaneConfig();

private slots:
	void runCalib();
	void proceedCalib();
	void saveCalib();

private:
	Ui::CPlaneConfig *m_ui;
};
