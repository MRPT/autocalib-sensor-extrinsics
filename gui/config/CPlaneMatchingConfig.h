#pragma once

#ifndef Q_MOC_RUN
#include <calib_solvers/CPlaneMatching.h>
#include <observation_tree/CObservationTreeModel.h>
#endif

#include <QWidget>

#include <functional>

namespace Ui {
class CPlaneMatchingConfig;
}

class CPlaneMatchingConfig : public QWidget
{
	Q_OBJECT

public:
	 CPlaneMatchingConfig(CObservationTreeModel *model, std::array<double,6> *init_calib, std::function<void(const std::string&)> updateFunction, QWidget *parent = 0);
	~CPlaneMatchingConfig();

private slots:
	void runCalib();
	void proceedCalib();
	void saveCalib();

private:
	Ui::CPlaneMatchingConfig *m_ui;
	CObservationTreeModel *m_model;
	std::array<double,6> *m_init_calib;
	std::function<void(const std::string&)> sendTextUpdate;
};
