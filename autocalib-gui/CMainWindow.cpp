#include "CMainWindow.h"
#include "ui_CMainWindow.h"

#include <QFileDialog>
#include <QDebug>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/math/ops_containers.h>

using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::system;
using namespace mrpt::math;

CMainWindow::CMainWindow(QWidget *parent) :
	QMainWindow(parent),
	m_ui(new Ui::CMainWindow)
{
	m_ui->setupUi(this);

	m_central_widget = new QWidget(this);
	this->setCentralWidget(m_central_widget);

	connect(m_ui->sensors_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(sensorsIndexChanged(int)));
	connect(m_ui->algo_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(algosIndexChanged(int)));
	connect(m_ui->load_rlog_button, SIGNAL(clicked(bool)), this, SLOT(openRawlog()));
	connect(m_ui->start_calib_button, SIGNAL(clicked(bool)), this, SLOT(startCalib()));
	connect(m_ui->continue_calib_button, SIGNAL(clicked(bool)), this, SLOT(continueCalib()));

	#ifndef NDEBUG
		m_ui->load_rlog_button->setDisabled(false);
		m_ui->continue_calib_button->setDisabled(false);
	#endif

	setWindowTitle("Automatic Calibration of Sensor Extrinsics");
	m_recent_file = m_settings.value("recent").toString();
	qDebug() << m_recent_file;
}

CMainWindow::~CMainWindow()
{
	m_settings.setValue("recent", m_recent_file);
	delete m_ui;
}

void CMainWindow::sensorsIndexChanged(int index)
{
   m_ui->algo_cbox->setDisabled(false);
}

void CMainWindow::algosIndexChanged(int index)
{
	m_ui->irx_sbox->setDisabled(false);
	m_ui->iry_sbox->setDisabled(false);
	m_ui->irz_sbox->setDisabled(false);
	m_ui->itx_sbox->setDisabled(false);
	m_ui->ity_sbox->setDisabled(false);
	m_ui->itz_sbox->setDisabled(false);
	m_ui->med_sbox->setDisabled(false);
	m_ui->load_rlog_button->setDisabled(false);
}

void CMainWindow::openRawlog()
{
	QString path;
	if(!m_recent_file.isEmpty())
	{
		QFileInfo fi(m_recent_file);
		path = fi.absolutePath();
	}

	QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"), path, tr("Rawlog Files (*.rawlog *.rawlog.gz)"));

	loadRawlog(file_name);
}

void CMainWindow::loadRawlog(QString file_name)
{
	if(file_name.isEmpty())
		return;

	qDebug() << file_name;
	m_recent_file = file_name;

	m_ui->statusBar->showMessage("Loading Rawlog...");

	CFileGZInputStream rawlog(file_name.toStdString());

	CSerializable::Ptr obj;

	bool read = true;
	while(read)
	{
		archiveFrom(rawlog) >> obj;

		if(!obj)
			read = false;
	}

	m_ui->start_calib_button->setDisabled(false);
	m_ui->continue_calib_button->setDisabled(false);
	m_ui->statusBar->showMessage("Rawlog loaded!");

}

void CMainWindow::startCalib()
{

}

void CMainWindow::continueCalib()
{

}
