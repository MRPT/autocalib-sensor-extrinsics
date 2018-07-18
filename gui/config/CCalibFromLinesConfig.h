#pragma once
#include <QWidget>

/**
 * Class for loading and storing parameters related to CCalibFromLines.
 */

namespace Ui {
class CCalibFromLinesConfig;
}

class CCalibFromLinesConfig : public QWidget
{
	Q_OBJECT

public:
	explicit CCalibFromLinesConfig(QWidget *parent = 0);
	~CCalibFromLinesConfig();

private slots:
	void startCalib();
	void proceedCalib();
	void saveCalib();

private:
	Ui::CCalibFromLinesConfig *m_ui;
};
