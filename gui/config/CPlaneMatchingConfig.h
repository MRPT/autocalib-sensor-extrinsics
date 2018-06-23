#pragma once

#include <QWidget>

namespace Ui {
class CPlaneMatchingConfig;
}

class CPlaneMatchingConfig : public QWidget
{
	Q_OBJECT

public:
	 CPlaneMatchingConfig(QWidget *parent = 0);
	~CPlaneMatchingConfig();

private slots:
	void startCalib();
	void proceedCalib();
	void saveCalib();

private:
	Ui::CPlaneMatchingConfig *m_ui;
};
