#include "CMainWindow.h"
#include "ui_CMainWindow.h"
#include "observation_tree/CObservationTreeModel.h"

#include <QFileDialog>
#include <QDebug>

CMainWindow::CMainWindow(QWidget *parent) :
	QMainWindow(parent),
	m_ui(new Ui::CMainWindow)
{
	m_ui->setupUi(this);

	connect(m_ui->sensors_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(sensorsIndexChanged(int)));
	connect(m_ui->algo_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(algosIndexChanged(int)));
	connect(m_ui->load_rlog_button, SIGNAL(clicked(bool)), this, SLOT(loadRawlog()));
	connect(m_ui->start_calib_button, SIGNAL(clicked(bool)), this, SLOT(startCalib()));
	connect(m_ui->continue_calib_button, SIGNAL(clicked(bool)), this, SLOT(continueCalib()));
	connect(m_ui->observations_treeview, SIGNAL(clicked(QModelIndex)), this, SLOT(itemClicked(QModelIndex)));

	#ifndef NDEBUG
		m_ui->load_rlog_button->setDisabled(false);
		m_ui->continue_calib_button->setDisabled(false);
	#endif

	setWindowTitle("Automatic Calibration of Sensor Extrinsics");
	m_recent_file = m_settings.value("recent").toString();
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

void CMainWindow::loadRawlog()
{
	QString path;

	if(!m_recent_file.isEmpty())
	{
		QFileInfo fi(m_recent_file);
		path = fi.absolutePath();
	}

	QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"), path, tr("Rawlog Files (*.rawlog *.rawlog.gz)"));

	if(file_name.isEmpty())
		return;

	m_ui->status_bar->showMessage("Loading Rawlog...");

	if(m_model)
		delete m_model;

	m_model = new CObservationTreeModel(file_name.toStdString(), m_ui->observations_treeview);
	m_ui->observations_treeview->setModel(m_model);

	m_recent_file = file_name;

	m_ui->start_calib_button->setDisabled(false);
	m_ui->continue_calib_button->setDisabled(false);
	m_ui->status_bar->showMessage("Rawlog loaded!");
}

void CMainWindow::startCalib()
{

}

void CMainWindow::continueCalib()
{

}

void CMainWindow::itemClicked(const QModelIndex &index)
{
	std::stringstream stream;

	m_model->observationData(index)->getDescriptionAsText(stream);
	m_ui->viewer_container->changeOutputText(QString::fromStdString(stream.str()));
}
