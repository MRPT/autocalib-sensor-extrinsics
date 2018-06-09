#pragma once

#include <calib_solvers/CPlaneMatching.h>
#include <observation_tree/CObservationTreeModel.h>
#include <ui_CViewerContainer.h>

#include <QWidget>

namespace Ui {
class CPlaneMatchingConfig;
}

class CPlaneMatchingConfig : public QWidget
{
	Q_OBJECT

public:
	 CPlaneMatchingConfig(CObservationTreeModel *model, std::array<double,6> *init_calib, Ui::CViewerContainer *viewer_ui, QWidget *parent = 0);
	~CPlaneMatchingConfig();

private slots:
	void runCalib();
	void proceedCalib();
	void saveCalib();

private:
	Ui::CPlaneMatchingConfig *m_ui;
	Ui::CViewerContainer *m_viewer_ui;
	CObservationTreeModel *m_model;
	std::array<double,6> *m_init_calib;
};
