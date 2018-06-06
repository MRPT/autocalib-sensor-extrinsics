#include <CMainWindow.h>
#include <ui_CMainWindow.h>
#include <observation_tree/CObservationTreeModel.h>
#include <config/CPlaneConfig.h>

#include <mrpt/maps/CSimplePointsMap.h>
#include <pcl/search/impl/search.hpp>

#include <QFileDialog>
#include <QSignalMapper>

CMainWindow::CMainWindow(QWidget *parent) :
	QMainWindow(parent),
	m_model(nullptr),
	m_ui(new Ui::CMainWindow)
{
	m_ui->setupUi(this);

	connect(m_ui->sensors_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(sensorsIndexChanged(int)));
	connect(m_ui->algo_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(algosIndexChanged(int)));
	connect(m_ui->open_rlog_button, SIGNAL(clicked(bool)), this, SLOT(openRawlog()));
	connect(m_ui->observations_treeview, SIGNAL(clicked(QModelIndex)), this, SLOT(itemClicked(QModelIndex)));

	QSignalMapper *signal_mapper = new QSignalMapper(this);
	signal_mapper->setMapping(m_ui->irx_sbox, 0);
	signal_mapper->setMapping(m_ui->iry_sbox, 1);
	signal_mapper->setMapping(m_ui->irz_sbox, 2);
	signal_mapper->setMapping(m_ui->itx_sbox, 3);
	signal_mapper->setMapping(m_ui->ity_sbox, 4);
	signal_mapper->setMapping(m_ui->itz_sbox, 5);

	connect(m_ui->irx_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));
	connect(m_ui->iry_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));
	connect(m_ui->irz_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));
	connect(m_ui->itx_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));
	connect(m_ui->ity_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));
	connect(m_ui->itz_sbox, SIGNAL(valueChanged(double)), signal_mapper, SLOT(map()));

	connect(signal_mapper, SIGNAL(mapped(int)), this, SLOT(initCalibChanged(int)));

	#ifndef NDEBUG
		m_ui->open_rlog_button->setDisabled(false);
		m_ui->algo_cbox->setDisabled(false);
	#endif

	setWindowTitle("Automatic Calibration of Sensor Extrinsics");
	//m_ui->viewer_container->changeOutputText(QString("Welcome to autocalib-sensor-extrinsics!"));
	m_recent_file = m_settings.value("recent").toString();

	m_init_calib.fill(0);
}

CMainWindow::~CMainWindow()
{
	m_settings.setValue("recent", m_recent_file);
	delete m_ui;
}

void CMainWindow::sensorsIndexChanged(int index)
{
	m_ui->algo_cbox->setDisabled(false);
	m_ui->irx_sbox->setDisabled(false);
	m_ui->iry_sbox->setDisabled(false);
	m_ui->irz_sbox->setDisabled(false);
	m_ui->itx_sbox->setDisabled(false);
	m_ui->ity_sbox->setDisabled(false);
	m_ui->itz_sbox->setDisabled(false);
	m_ui->med_sbox->setDisabled(false);
	m_ui->open_rlog_button->setDisabled(false);
}

void CMainWindow::algosIndexChanged(int index)
{
	switch(index)
	{
		case 0:
		{
			if(m_config_widget)
				m_config_widget.reset();
		}
		break;

		case 1:
		{
			m_config_widget = std::make_shared<CPlaneConfig>(new CPlaneConfig());
			qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(1, m_config_widget.get());
		}
		break;
	}
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

	if(file_name.isEmpty())
		return;

	m_ui->status_bar->showMessage("Loading Rawlog...");

	if(m_model)
		delete m_model;

	m_model = new CObservationTreeModel(file_name.toStdString(), m_ui->observations_treeview);
	m_ui->observations_treeview->setModel(m_model);

	m_recent_file = file_name;

	m_ui->status_bar->showMessage("Rawlog loaded!");
}

void CMainWindow::itemClicked(const QModelIndex &index)
{
	if(index.isValid())
	{
		std::stringstream stream;
		mrpt::obs::CObservation3DRangeScan::Ptr obs_item;
		mrpt::maps::CPointsMap::Ptr map;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		int viewer_id = 3;
		std::string viewer_text;

		// check if observation was clicked
		if((index.parent()).isValid())
		{
			obs_item = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(m_model->observationData(index));

			obs_item->getDescriptionAsText(stream);
			m_ui->viewer_container->changeOutputText(QString::fromStdString(stream.str()));

			map = mrpt::make_aligned_shared<mrpt::maps::CSimplePointsMap>();
			map->insertObservation(obs_item.get());
			map->getPCLPointCloud(*cloud);

			viewer_id = m_model->m_obs_labels.indexOf(QString::fromStdString(obs_item->sensorLabel + " : " + obs_item->GetRuntimeClass()->className)) + 1;
			viewer_text = m_model->data(index.parent()).toString().toStdString();
			m_ui->viewer_container->updateViewer(viewer_id, cloud, viewer_text);
		}

		// else parent item was clicked
		else
		{
			CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

			for(int i = 0; i < item->childCount(); i++)
			{
				obs_item = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>((item->child(i))->getObservation());
				obs_item->getDescriptionAsText(stream);
				stream << "-----------------------------------------------------------------------------------------------------------------------------\n";

				map = mrpt::make_aligned_shared<mrpt::maps::CSimplePointsMap>();
				map->insertObservation(obs_item.get());
				map->getPCLPointCloud(*cloud);

				viewer_id = m_model->m_obs_labels.indexOf(QString::fromStdString(obs_item->sensorLabel + " : " + obs_item->GetRuntimeClass()->className)) + 1;
				viewer_text = (m_model->data(index)).toString().toStdString();
				m_ui->viewer_container->updateViewer(viewer_id, cloud, viewer_text);
			}

			m_ui->viewer_container->changeOutputText(QString::fromStdString(stream.str()));
		}
	}
}

void CMainWindow::initCalibChanged(int index)
{
	std::cout << index;
}

std::array<double,6> CMainWindow::getInitCalib()
{
	return m_init_calib;
}
