#pragma once

#include <observation_tree/CObservationTreeModel.h>

#include <QMainWindow>
#include <QSettings>

class CPlaneMatching;

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit CMainWindow(QWidget *parent = 0);
	~CMainWindow();
	std::array<double, 6> getInitCalib();

private slots:
	void sensorsIndexChanged(int index);
	void algosIndexChanged(int index);
	void openRawlog();
	void itemClicked(const QModelIndex &);
	void initCalibChanged(int index);

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	QSettings m_settings;
	QString m_recent_file;

	CObservationTreeModel *m_model;

	std::shared_ptr<QWidget> m_config_widget;
	std::array<double,6> m_init_calib;
};
