#pragma once
#include <QWidget>

namespace Ui {
class CLineMatchingConfig;
}

class CLineMatchingConfig : public QWidget
{
	Q_OBJECT

public:
	explicit CLineMatchingConfig(QWidget *parent = 0);
	~CLineMatchingConfig();

private slots:
	void startCalib();
	void proceedCalib();
	void saveCalib();


private:
	Ui::CLineMatchingConfig *m_ui;
};
