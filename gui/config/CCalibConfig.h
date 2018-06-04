#pragma once

#include <QWidget>

namespace Ui {
class CCalibConfig;
}

class CCalibConfig : public QWidget
{
	Q_OBJECT

public:
	explicit CCalibConfig(QWidget *parent = 0);
	~CCalibConfig();

private:
	Ui::CCalibConfig *ui;
};
