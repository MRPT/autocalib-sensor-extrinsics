#pragma once

#include <observation_tree/CObservationTreeModel.h>
#include <calib_solvers/TPlaneMatchingParams.h>
#include <calib_solvers/CPlaneMatching.h>
#include <calib_solvers/CLineMatching.h>

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
	void runPlaneMatchingCalib(TPlaneMatchingParams params);
	void runLineMatchingCalib();

private slots:
	void sensorsIndexChanged(int index);
	void algosIndexChanged(int index);
	void loadRawlog();
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
	CPlaneMatching *m_plane_matching; /* to be replaced by a generic calib algo object */
	CLineMatching *m_line_matching;
	bool m_calib_started;
};
