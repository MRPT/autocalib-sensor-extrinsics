#pragma once

#include <QMainWindow>
#include <QSettings>

class QGroupBox;
class QComboBox;
class QWidget;
class QPushButton;

namespace Ui {
class CMainWindow;
}

class CMainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit CMainWindow(QWidget *parent = 0);
	void loadRawlog(QString);
	~CMainWindow();

private slots:
	void sensorsIndexChanged(int index);
	void algosIndexChanged(int index);
	void openRawlog();
	void startCalib();
	void continueCalib();

private:
	Ui::CMainWindow *m_ui;
	QWidget *m_central_widget;

	QSettings m_settings;
	QString m_recent_file;

};
