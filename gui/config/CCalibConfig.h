#ifndef CCALIBCONFIG_H
#define CCALIBCONFIG_H

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

#endif // CCALIBCONFIG_H
