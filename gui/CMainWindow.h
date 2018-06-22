#pragma once

#ifndef Q_MOC_RUN
#include <observation_tree/CObservationTreeModel.h>
#include <calib_solvers/TPlaneMatchingParams.h>
#endif

#include <QMainWindow>
#include <QSettings>

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit CMainWindow(QWidget *parent = 0);
	~CMainWindow();
	void runCalib(TPlaneMatchingParams params);

private slots:
	void sensorsIndexChanged(int index);
	void algosIndexChanged(int index);
	void openRawlog();
	void itemClicked(const QModelIndex &);
	void initCalibChanged(double value);

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	QSettings m_settings;
	QString m_recent_file;

	std::array<double,6> m_init_calib;
	CObservationTreeModel *m_model;

	std::shared_ptr<QWidget> m_config_widget;
};
