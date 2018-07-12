#include <CMainWindow.h>
#include <ui_CMainWindow.h>
#include <observation_tree/CObservationTreeModel.h>
#include <config/CPlaneMatchingConfig.h>
#include <config/CLineMatchingConfig.h>
#include <core_gui/CCalibFromPlanesGui.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/PCL_adapters.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <pcl/search/impl/search.hpp>

#include <QFileDialog>
#include <QSpinBox>
#include <QDebug>

using namespace mrpt::obs;

CMainWindow::CMainWindow(const std::string &config_file_name, QWidget *parent) :
    QMainWindow(parent),
    m_config_file(config_file_name),
	m_model(nullptr),
    m_sync_model(nullptr),
	m_ui(new Ui::CMainWindow)
{
	m_ui->setupUi(this);

	m_ui->irx_sbox->setValue(m_config_file.read_double("initial_calibration", "irx", 0.0, true));
	m_ui->iry_sbox->setValue(m_config_file.read_double("initial_calibration", "iry", 0.0, true));
	m_ui->irz_sbox->setValue(m_config_file.read_double("initial_calibration", "irz", 0.0, true));
	m_ui->itx_sbox->setValue(m_config_file.read_double("initial_calibration", "itx", 0.0, true));
	m_ui->ity_sbox->setValue(m_config_file.read_double("initial_calibration", "ity", 0.0, true));
	m_ui->itz_sbox->setValue(m_config_file.read_double("initial_calibration", "itz", 0.0, true));

	m_ui->observations_delay_sbox->setValue(m_config_file.read_int("grouping_observations", "max_delay", 30, true));

	connect(m_ui->load_rlog_button, SIGNAL(clicked(bool)), this, SLOT(loadRawlog()));
	connect(m_ui->algo_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(algosIndexChanged(int)));
	connect(m_ui->observations_treeview, SIGNAL(clicked(QModelIndex)), this, SLOT(listItemClicked(QModelIndex)));
	connect(m_ui->sync_observations_button, SIGNAL(clicked(bool)), this, SLOT(syncObservationsClicked()));
	connect(m_ui->grouped_observations_treeview, SIGNAL(clicked(QModelIndex)), this, SLOT(treeItemClicked(QModelIndex)));

	connect(m_ui->irx_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->iry_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->irz_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->itx_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->ity_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->itz_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));

	setWindowTitle("Automatic Calibration of Sensor Extrinsics");
	m_calib_started = false;
	m_calib_from_planes_gui = nullptr;
	m_calib_from_lines_gui = nullptr;
	m_ui->viewer_container->updateText("Welcome to autocalib-sensor-extrinsics!");
	m_ui->viewer_container->updateText("Set your initial (rough) calibration values and load your rawlog file to get started.");
	m_recent_file = m_settings.value("recent").toString();

	m_init_calib[0] = m_ui->irx_sbox->value();
	m_init_calib[1] = m_ui->iry_sbox->value();
	m_init_calib[2] = m_ui->irz_sbox->value();
	m_init_calib[3] = m_ui->itz_sbox->value();
	m_init_calib[4] = m_ui->ity_sbox->value();
	m_init_calib[5] = m_ui->itz_sbox->value();
}

CMainWindow::~CMainWindow()
{
	m_settings.setValue("recent", m_recent_file);
	delete m_ui;
}

void CMainWindow::algosIndexChanged(int index)
{
	switch(index)
	{
	case 0:
	{
		if(m_config_widget)
			m_config_widget.reset();
		break;
	}

	case 1:
	{
		m_config_widget = std::make_shared<CPlaneMatchingConfig>(m_config_file);
		qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(3, m_config_widget.get());
		break;
	}

	case 2:
	{
		m_config_widget = std::make_shared<CLineMatchingConfig>();
		qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(3, m_config_widget.get());
		break;
	}
	}
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

	// To ensure all options are disabled when a new rawlog is loaded again.
	m_ui->observations_treeview->setDisabled(true);
	m_ui->observations_description_textbrowser->setDisabled(true);
	m_ui->observations_delay_sbox->setDisabled(true);
	m_ui->sensors_selection_list->setDisabled(true);
	m_ui->sync_observations_button->setDisabled(true);
	m_ui->grouped_observations_treeview->setDisabled(true);
	m_ui->algo_cbox->setDisabled(true);
	m_recent_file = file_name;

	if(m_model)
		delete m_model;

	m_model = new CObservationTreeModel(m_ui->observations_treeview);
	m_model->addTextObserver(m_ui->viewer_container);
	m_model->loadModel(file_name.toStdString());

	if((m_model->getRootItem()) != nullptr)
	{
		m_ui->observations_treeview->setDisabled(false);
		m_ui->observations_treeview->setModel(m_model);
		m_ui->status_bar->showMessage("Rawlog loaded!");
		m_ui->viewer_container->updateText("Select the calibration algorithm to continue.");
		m_ui->observations_description_textbrowser->setDisabled(false);
		m_ui->sensors_selection_list->setDisabled(false);
		m_ui->observations_delay_sbox->setDisabled(false);
		m_ui->sync_observations_button->setDisabled(false);

		QStringList sensor_labels = m_model->getSensorLabels();

		for(size_t i = 0; i < sensor_labels.size(); i++)
		{
			QListWidgetItem *item = new QListWidgetItem;
			item->setText(sensor_labels[i]);
			item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
			item->setCheckState(Qt::Checked);
			m_ui->sensors_selection_list->insertItem(i, item);
		}
	}

	else
	{
		m_ui->status_bar->showMessage("Loading aborted!");
		m_ui->viewer_container->updateText("Loading was aborted. Try again.");
	}
}

void CMainWindow::syncObservationsClicked()
{
	QStringList selected_sensor_labels;
	QListWidgetItem *item;

	m_ui->grouped_observations_treeview->setDisabled(true);

	for(size_t i = 0; i < m_ui->sensors_selection_list->count(); i++)
	{
		item = m_ui->sensors_selection_list->item(i);
		if(item->checkState() == Qt::Checked)
			selected_sensor_labels.push_back(item->text());
	}

	if(!(selected_sensor_labels.size() > 1))
		m_sync_model = nullptr;

	else
	{
		if(m_sync_model)
		{
			delete m_sync_model;
		}

		// creating a copy of the model
		m_sync_model = new CObservationTreeModel(m_ui->grouped_observations_treeview);
		m_sync_model->setRootItem(new CObservationTreeItem(QString("root")));

		for(size_t i = 0; i < m_model->getRootItem()->childCount(); i++)
		{
			m_sync_model->getRootItem()->appendChild(m_model->getRootItem()->child(i));
		}

		m_sync_model->syncObservations(selected_sensor_labels, m_ui->observations_delay_sbox->value());

		if(m_sync_model->getRootItem()->childCount() > 0)
		{
			m_ui->grouped_observations_treeview->setDisabled(false);
			m_ui->grouped_observations_treeview->setModel(m_sync_model);
			m_ui->algo_cbox->setDisabled(false);
		}
	}
}

void CMainWindow::listItemClicked(const QModelIndex &index)
{
    if(index.isValid())
    {
        CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

        std::stringstream update_stream;
        std::string viewer_text;
        int viewer_id, sensor_id;

		CObservation3DRangeScan::Ptr obs_item;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		mrpt::img::CImage image;

		T3DPointsProjectionParams projection_params;
		projection_params.MAKE_DENSE = false;
		projection_params.MAKE_ORGANIZED = false;

        obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_model->observationData(index));
        obs_item->getDescriptionAsText(update_stream);
		obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
        cloud->is_dense = false;

		image = obs_item->intensityImage;

        sensor_id = m_model->getSensorLabels().indexOf(QString::fromStdString(obs_item->sensorLabel)) + 1;
        viewer_id = sensor_id;

        viewer_text = (m_model->data(index)).toString().toStdString();

		m_ui->viewer_container->updateCloudViewer(viewer_id, cloud, viewer_text);
		m_ui->viewer_container->updateImageViewer(viewer_id, image);

		//if(m_calib_started && (m_calib_from_planes_gui != nullptr))
		    //m_calib_from_planes_gui->publishPlaneCloud(item->parentItem()->row(), item->row(), sensor_id);

		m_ui->observations_description_textbrowser->setText(QString::fromStdString(update_stream.str()));
	  }
}

void CMainWindow::treeItemClicked(const QModelIndex &index)
{
	if(index.isValid())
	{
		CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

		std::stringstream update_stream;
		std::string viewer_text;
		int viewer_id, sensor_id;

		CObservation3DRangeScan::Ptr obs_item;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		mrpt::img::CImage image;

		T3DPointsProjectionParams projection_params;
		projection_params.MAKE_DENSE = false;
		projection_params.MAKE_ORGANIZED = false;

		//if single-item was clicked
		if((index.parent()).isValid())
		{
			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_model->observationData(index));
			obs_item->getDescriptionAsText(update_stream);
			obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
			cloud->is_dense = false;

			image = obs_item->intensityImage;

			sensor_id = m_model->getSensorLabels().indexOf(QString::fromStdString(obs_item->sensorLabel)) + 1;
			viewer_id = sensor_id;

			viewer_text = (m_model->data(index.parent())).toString().toStdString();

			m_ui->viewer_container->updateCloudViewer(viewer_id, cloud, viewer_text);
			m_ui->viewer_container->updateImageViewer(viewer_id, image);

			if(m_calib_started && (m_calib_from_planes_gui != nullptr))
				m_calib_from_planes_gui->publishPlaneCloud(item->parentItem()->row(), item->row(), sensor_id);
		}

		// else set-item was clicked
		else
		{
			for(int i = 0; i < item->childCount(); i++)
			{
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(item->child(i)->getObservation());
				obs_item->getDescriptionAsText(update_stream);
				obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
				cloud->is_dense = false;

				image = obs_item->intensityImage;

				sensor_id = m_model->getSensorLabels().indexOf(QString::fromStdString(obs_item->sensorLabel)) + 1;
				viewer_id = sensor_id;

				viewer_text = (m_model->data(index)).toString().toStdString();
				update_stream << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";

				m_ui->viewer_container->updateCloudViewer(viewer_id, cloud, viewer_text);
				m_ui->viewer_container->updateImageViewer(viewer_id, image);

				if(m_calib_started && (m_calib_from_planes_gui != nullptr))
					m_calib_from_planes_gui->publishPlaneCloud(item->row(), i, sensor_id);
			}
		}
    
        m_ui->observations_description_textbrowser->setText(QString::fromStdString(update_stream.str()));
    }
}

void CMainWindow::initCalibChanged(double value)
{
	QSpinBox *sbox = (QSpinBox*)sender();

	if(sbox->accessibleName() == QString("irx"))
		m_init_calib[0] = value;
	else if(sbox->accessibleName() == QString("iry"))
		m_init_calib[1] = value;
	else if(sbox->accessibleName() == QString("irz"))
		m_init_calib[2] = value;
	else if(sbox->accessibleName() == QString("itx"))
		m_init_calib[3] = value;
	else if(sbox->accessibleName() == QString("ity"))
		m_init_calib[4] = value;
	else if(sbox->accessibleName() == QString("itz"))
		m_init_calib[5] = value;
}

void CMainWindow::runCalibFromPlanes(TPlaneMatchingParams params)
{
	if(m_sync_model != nullptr)
	{
		m_calib_from_lines_gui = nullptr;

		m_calib_from_planes_gui = new CCalibFromPlanesGui(m_sync_model, params);
		m_calib_from_planes_gui->addTextObserver(m_ui->viewer_container);
		m_calib_from_planes_gui->addPlanesObserver(m_ui->viewer_container);
		m_calib_from_planes_gui->extractPlanes();

		m_calib_started = true;
	}

	else
		m_ui->viewer_container->updateText("Error. Choose atleast two sensors!");
}

void CMainWindow::runCalibFromLines()
{
	if(m_sync_model != nullptr)
	{
		m_calib_from_planes_gui = nullptr;

		m_calib_from_lines_gui = new CCalibFromLinesGui(m_sync_model);
		//m_line_matching->extractLines();

		m_calib_started = true;
	}

	else
		m_ui->viewer_container->updateText("Error. Choose atleast two sensors!");
}

void CMainWindow::saveParams()
{
	m_config_file.write<double>("initial_calibration", "irx", m_ui->irx_sbox->value());
	m_config_file.write<double>("initial_calibration", "iry", m_ui->iry_sbox->value());
	m_config_file.write<double>("initial_calibration", "irz", m_ui->irz_sbox->value());
	m_config_file.write<double>("initial_calibration", "itx", m_ui->itx_sbox->value());
	m_config_file.write<double>("initial_calibration", "ity", m_ui->ity_sbox->value());
	m_config_file.write<double>("initial_calibration", "itz", m_ui->itz_sbox->value());

	m_config_file.write<int>("grouping_observations", "max_delay", m_ui->observations_delay_sbox->value());

	m_config_file.writeNow();

	m_ui->status_bar->showMessage("Parameters saved!");
}
