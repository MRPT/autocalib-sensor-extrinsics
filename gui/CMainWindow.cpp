#include <CMainWindow.h>
#include <ui_CMainWindow.h>
#include <observation_tree/CObservationTreeGui.h>
#include <Utils.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/PCL_adapters.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/system/CTicTac.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/common/transforms.h>

#include <QFileDialog>
#include <QSpinBox>
#include <QDebug>

#include <thread>

using namespace mrpt::obs;
using namespace mrpt::system;

CMainWindow::CMainWindow(QWidget *parent) :
    QMainWindow(parent),
	m_model(nullptr),
    m_sync_model(nullptr),
	m_ui(new Ui::CMainWindow)
{
	m_ui->setupUi(this);

	connect(m_ui->config_file_select_button, SIGNAL(clicked(bool)), this, SLOT(loadConfigFile()));
	connect(m_ui->rlog_file_select_button, SIGNAL(clicked(bool)), this, SLOT(selectRawlogFile()));
	connect(m_ui->load_rlog_button, SIGNAL(clicked(bool)), this, SLOT(loadRawlog()));
	connect(m_ui->sensor_cbox, SIGNAL(currentIndexChanged(int)), this, SLOT(sensorIndexChanged(int)));
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
	connect(m_ui->angle_uncertain_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));
	connect(m_ui->distance_uncertain_sbox, SIGNAL(valueChanged(double)), this, SLOT(initCalibChanged(double)));

	setWindowTitle("Automatic Calibration of Sensor Extrinsics");
	m_calib_from_planes_gui = nullptr;
	m_calib_from_lines_gui = nullptr;
	m_ui->viewer_container->updateText("Welcome to autocalib-sensor-extrinsics!");
	m_ui->viewer_container->updateText("Load your config file to get started.");
	m_recent_rlog_path = m_settings.value("recent_rlog").toString();
	m_recent_config_path = m_settings.value("recent_config").toString();

	m_calib_from_planes_config_widget = std::make_shared<CCalibFromPlanesConfig>();
	qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(8, m_calib_from_planes_config_widget.get());

	m_calib_from_lines_config_widget = std::make_shared<CCalibFromLinesConfig>();
	qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(9, m_calib_from_lines_config_widget.get());

	m_calib_from_planes_config_widget.get()->hide();
	m_calib_from_lines_config_widget.get()->hide();
}

CMainWindow::~CMainWindow()
{
	m_settings.setValue("recent_rlog", m_recent_rlog_path);
	m_settings.setValue("recent_config", m_recent_config_path);
	delete m_ui;
}

void CMainWindow::loadConfigFile()
{
	m_ui->rlog_file_line_edit->setDisabled(true);
	m_ui->rlog_file_select_button->setDisabled(true);
	m_ui->load_rlog_button->setDisabled(true);

	QString path;
	if(!m_recent_config_path.isEmpty())
	{
		QFileInfo fi(m_recent_config_path);
		path = fi.absolutePath();
	}

	else
		path = QFileInfo(QString::fromStdString(PROJECT_SOURCE_PATH + std::string("/config_files/"))).absolutePath();

	path = QFileDialog::getOpenFileName(this, tr("Load Configuration File"), path, tr("Configuration Files (*.ini)"));
	if(!path.isEmpty())
	{
		m_recent_config_path = path;
		m_ui->config_file_line_edit->setText(path);
		m_config_file.setFileName(path.toStdString());
		m_ui->rlog_file_line_edit->setText(QString::fromStdString(m_config_file.read_string("rawlog", "path", "")));
		m_ui->rlog_file_line_edit->setDisabled(false);
		m_ui->rlog_file_select_button->setDisabled(false);
		if(!m_ui->rlog_file_line_edit->text().isEmpty())
			m_ui->load_rlog_button->setDisabled(false);

		m_calib_from_planes_config_widget->setConfig(m_config_file);
		m_calib_from_lines_config_widget->setConfig(m_config_file);
	}
}

void CMainWindow::selectRawlogFile()
{
	m_ui->load_rlog_button->setDisabled(true);
	QString path;
	if(!m_recent_rlog_path.isEmpty())
	{
		QFileInfo fi(m_recent_rlog_path);
		path = fi.absolutePath();
	}

	path = QFileDialog::getOpenFileName(this, tr("Select Rawlog File"), path, tr("Rawlog Files (*.rawlog *.rawlog.gz)"));
	if(!path.isEmpty())
	{
		m_recent_rlog_path = path;
		m_ui->rlog_file_line_edit->setText(path);
		m_ui->load_rlog_button->setDisabled(false);
	}
}

void CMainWindow::loadRawlog()
{
	// To ensure all options are disabled when a new rawlog is loaded again.
	m_ui->observations_treeview->setDisabled(true);
	m_ui->observations_description_textbrowser->setDisabled(true);
	m_ui->observations_delay_sbox->setDisabled(true);
	m_ui->sensors_selection_list->setDisabled(true);
	m_ui->sync_observations_button->setDisabled(true);
	m_ui->grouped_observations_treeview->setDisabled(true);
	m_ui->algo_cbox->setDisabled(true);

	QString rlog_path;
	rlog_path = m_ui->rlog_file_line_edit->text();

	if(rlog_path.isEmpty())
	{
		m_ui->status_bar->showMessage("Load error!");
		m_ui->viewer_container->updateText("Please select the rawlog file first.");
		return;
	}

	if(!QFile(rlog_path).exists())
	{
		m_ui->status_bar->showMessage("Load error!");
		m_ui->viewer_container->updateText("File not found. Please check your rawlog path and try again.");
		return;
	}

	m_ui->status_bar->showMessage("Loading Rawlog...");

	if(m_model)
		delete m_model;

	CTicTac stop_watch;
	double time_to_load;

	m_model = new CObservationTreeGui(rlog_path.toStdString(), m_config_file, m_ui->observations_treeview);
	m_model->addTextObserver(m_ui->viewer_container);

	stop_watch.Tic();
	m_model->loadTree();
	time_to_load = stop_watch.Tac();

	if((m_model->getRootItem()) != nullptr && m_model->getRootItem()->childCount() > 0)
	{
		m_ui->observations_treeview->setDisabled(false);
		m_ui->observations_treeview->setModel(m_model);
		m_ui->status_bar->showMessage("Rawlog loaded!");
		m_ui->viewer_container->updateText("Select the calibration algorithm to continue.");
		m_ui->observations_description_textbrowser->setDisabled(false);
		m_ui->sensors_selection_list->setDisabled(false);
		m_ui->observations_delay_sbox->setDisabled(false);
		m_ui->sync_observations_button->setDisabled(false);

		m_ui->observations_delay_sbox->setValue(m_config_file.read_int("grouping_observations", "max_delay", 30, true));

		std::vector<std::string> sensor_labels = m_model->getSensorLabels();

		for(size_t i = 0; i < sensor_labels.size(); i++)
		{
			QListWidgetItem *item = new QListWidgetItem;
			item->setText(QString::fromStdString(sensor_labels[i]));
			item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
			item->setCheckState(Qt::Checked);
			m_ui->sensors_selection_list->insertItem(i, item);
		}

		std::string stats_string;
		stats_string = "RAWLOG STATS";
		stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";
		stats_string += "\nNumber of observations loaded: " + std::to_string(m_model->getObsCount());
		stats_string += "\nNumber of unique sensors found in rawlog: " + std::to_string(m_model->getSensorLabels().size());
		stats_string += "\n\nSummary of sensors found in rawlog:";
		stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";

		for(size_t i = 0; i < m_model->getSensorLabels().size(); i++)
		{
			stats_string += "\nSensor #" + std::to_string(i);
			stats_string += "\nSensor label : Class :: " + m_model->getSensorLabels()[i];
			stats_string += "\nNumber of observations: " + std::to_string(m_model->getCountOfLabel()[i]) + "\n";
		}

		m_ui->viewer_container->updateText(stats_string);
		m_ui->viewer_container->resetViewers(m_model->getSensorLabels(), m_model->getSensorPoses());
	}

	else
	{
		m_ui->status_bar->showMessage("Loading aborted!");
		m_ui->viewer_container->updateText("Loading was aborted. Please check your rawlog file and try again.");
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
		mrpt::img::CImage::Ptr image(new mrpt::img::CImage());

		obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_model->getItem(index)->getObservation());
		obs_item->getDescriptionAsText(update_stream);
		image = std::make_shared<mrpt::img::CImage>(obs_item->intensityImage);
		sensor_id = utils::findItemIndexIn(m_model->getSensorLabels(), obs_item->sensorLabel);

		viewer_text = (m_model->data(index)).toString().toStdString();

		m_ui->viewer_container->updateImageViewer(sensor_id, image);
		m_ui->observations_description_textbrowser->setText(QString::fromStdString(update_stream.str()));

		if(item->cloud() != nullptr)
			m_ui->viewer_container->updateCloudViewer(sensor_id, item->cloud(), viewer_text);

		else
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

			T3DPointsProjectionParams projection_params;
			projection_params.MAKE_DENSE = false;
			projection_params.MAKE_ORGANIZED = true;

			obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
			cloud->is_dense = false;

			m_ui->viewer_container->updateCloudViewer(sensor_id, cloud, viewer_text);

			item->setCloud(cloud);
		}
	}
}

void CMainWindow::syncObservationsClicked()
{
	std::vector<std::string> selected_sensor_labels;
	QListWidgetItem *item;

	m_ui->grouped_observations_treeview->setDisabled(true);
	m_ui->observations_treeview->setDisabled(false);
	m_ui->sensor_cbox->setDisabled(true);
	m_ui->irx_sbox->setDisabled(true);
	m_ui->iry_sbox->setDisabled(true);
	m_ui->irz_sbox->setDisabled(true);
	m_ui->itx_sbox->setDisabled(true);
	m_ui->ity_sbox->setDisabled(true);
	m_ui->itz_sbox->setDisabled(true);
	m_ui->angle_uncertain_sbox->setDisabled(true);
	m_ui->distance_uncertain_sbox->setDisabled(true);

	for(size_t i = 0; i < m_ui->sensors_selection_list->count(); i++)
	{
		item = m_ui->sensors_selection_list->item(i);
		if(item->checkState() == Qt::Checked)
			selected_sensor_labels.push_back(item->text().toStdString());
	}

	if(!(selected_sensor_labels.size() > 1))
	{
		m_sync_model = nullptr;
		m_ui->viewer_container->updateText("Error. Choose atleast two sensors!");
	}

	else
	{
		if(m_sync_model)
			delete m_sync_model;

		// creating a copy of the model
		m_sync_model = new CObservationTreeGui(m_model->getRawlogPath(), m_config_file, m_ui->grouped_observations_treeview);

		for(size_t i = 0; i < m_model->getRootItem()->childCount(); i++)
		{
			m_sync_model->getRootItem()->appendChild(m_model->getRootItem()->child(i));
		}

		m_sync_model->syncObservations(selected_sensor_labels, m_ui->observations_delay_sbox->value());

		if(m_sync_model->getRootItem()->childCount() > 0)
		{
			m_ui->observations_treeview->setDisabled(true);
			m_ui->grouped_observations_treeview->setDisabled(false);
			m_ui->grouped_observations_treeview->setModel(m_sync_model);
			m_ui->algo_cbox->setDisabled(false);

			m_ui->sensor_cbox->setDisabled(false);
			m_ui->irx_sbox->setDisabled(false);
			m_ui->iry_sbox->setDisabled(false);
			m_ui->irz_sbox->setDisabled(false);
			m_ui->itx_sbox->setDisabled(false);
			m_ui->ity_sbox->setDisabled(false);
			m_ui->itz_sbox->setDisabled(false);
			m_ui->angle_uncertain_sbox->setDisabled(false);
			m_ui->distance_uncertain_sbox->setDisabled(false);

			for(size_t i = 0; i < selected_sensor_labels.size(); i++)
			{
				QListWidgetItem *item = new QListWidgetItem;
				item->setText(QString::fromStdString(selected_sensor_labels[i]));
				item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
				item->setCheckState(Qt::Checked);
				m_ui->sensor_cbox->insertItem(i, QString::fromStdString(selected_sensor_labels[i]));
			}

			std::string stats_string;
			stats_string = "GROUPING STATS";
			stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";
			stats_string += "\nNumber of observation sets formed: " + std::to_string(m_sync_model->getRootItem()->childCount());
			stats_string += "\n\nSummary of sensors used:";
			stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";

			for(size_t i = 0; i < selected_sensor_labels.size(); i++)
			{
				stats_string += "\nSensor #" + std::to_string(i);
				stats_string += "\nSensor label : Class :: " + selected_sensor_labels[i] + " : "
				        + m_model->getRootItem()->child(m_sync_model->getSyncIndices()[i][0])->getObservation()->GetRuntimeClass()->className;
				stats_string += "\nNumber of observations: " + std::to_string(m_sync_model->getSyncIndices()[i].size()) + "\n";
			}

			m_ui->viewer_container->updateText(stats_string);
			m_ui->viewer_container->resetViewers(selected_sensor_labels, m_sync_model->getSensorPoses());
			m_ui->viewer_container->observationsSynced = true;
		}

		else
			m_ui->viewer_container->updateText("Zero observations grouped.");
	}
}

void CMainWindow::sensorIndexChanged(int index)
{
	Eigen::Matrix4f rt = m_sync_model->getSensorPoses()[index];
	Eigen::Vector2f uncertain = m_sync_model->getSensorUncertainties()[index];

	Eigen::Matrix<float,3,1> rvec = utils::getRotations(rt);
	Eigen::Matrix<float,3,1> tvec = utils::getTranslations(rt);

	m_ui->irx_sbox->blockSignals(true);
	m_ui->iry_sbox->blockSignals(true);
	m_ui->irz_sbox->blockSignals(true);
	m_ui->itx_sbox->blockSignals(true);
	m_ui->ity_sbox->blockSignals(true);
	m_ui->itz_sbox->blockSignals(true);
	m_ui->angle_uncertain_sbox->blockSignals(true);
	m_ui->distance_uncertain_sbox->blockSignals(true);

	m_ui->irx_sbox->setValue(rvec(0,0));
	m_ui->iry_sbox->setValue(rvec(1,0));
	m_ui->irz_sbox->setValue(rvec(2,0));
	m_ui->itx_sbox->setValue(tvec(0,0));
	m_ui->ity_sbox->setValue(tvec(1,0));
	m_ui->itz_sbox->setValue(tvec(2,0));
	m_ui->angle_uncertain_sbox->setValue(uncertain(0));
	m_ui->distance_uncertain_sbox->setValue(uncertain(1));

	m_ui->irx_sbox->blockSignals(false);
	m_ui->iry_sbox->blockSignals(false);
	m_ui->irz_sbox->blockSignals(false);
	m_ui->itx_sbox->blockSignals(false);
	m_ui->ity_sbox->blockSignals(false);
	m_ui->itz_sbox->blockSignals(false);
	m_ui->angle_uncertain_sbox->blockSignals(false);
	m_ui->distance_uncertain_sbox->blockSignals(false);
}

void CMainWindow::initCalibChanged(double value)
{
	QDoubleSpinBox *sbox = (QDoubleSpinBox*)sender();

	int sensor_index = m_ui->sensor_cbox->currentIndex();

	if(sbox->accessibleName() == QString("angle_uncertain_sbox") || sbox->accessibleName() == QString("distance_uncertain_sbox"))
	{
		Eigen::Vector2f uncertainty;
		uncertainty(0) = m_ui->angle_uncertain_sbox->value();
		uncertainty(1) = m_ui->distance_uncertain_sbox->value();
		m_sync_model->setSensorUncertainty(uncertainty, sensor_index);
	}

	else
	{
		std::vector<Eigen::Matrix4f> sensor_poses = m_sync_model->getSensorPoses();

		Eigen::Matrix<float,3,1> rotation_angles;
		Eigen::Matrix<float,3,1> translations;

		rotation_angles(0) = m_ui->irx_sbox->value() * (M_PI/180);
		rotation_angles(1) = m_ui->iry_sbox->value() * (M_PI/180);
		rotation_angles(2) = m_ui->irz_sbox->value() * (M_PI/180);
		sensor_poses[sensor_index].block(0,0,3,3) = utils::getRotationMatrix(rotation_angles);

		translations(0) = m_ui->itx_sbox->value();
		translations(1) = m_ui->ity_sbox->value();
		translations(2) = m_ui->itz_sbox->value();
		sensor_poses[sensor_index].block(0,3,3,1) = translations;

		m_sync_model->setSensorPose(sensor_poses[sensor_index], sensor_index);
	}
}

void CMainWindow::treeItemClicked(const QModelIndex &index)
{
	if(index.isValid())
	{
		CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

		std::stringstream update_stream;
		std::string viewer_text;
		int sensor_id;

		CObservation3DRangeScan::Ptr obs_item;
		size_t sync_obs_id;
		mrpt::img::CImage::Ptr image(new mrpt::img::CImage());

		//if single-item was clicked
		if((index.parent()).isValid())
		{
			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_sync_model->getItem(index)->getObservation());
			obs_item->getDescriptionAsText(update_stream);
			image = std::make_shared<mrpt::img::CImage>(obs_item->intensityImage);

			sensor_id = utils::findItemIndexIn(m_sync_model->getSensorLabels(), obs_item->sensorLabel);
			sync_obs_id = utils::findItemIndexIn(m_sync_model->getSyncIndices()[sensor_id], item->getPriorIndex());
			viewer_text = (m_sync_model->data(index.parent())).toString().toStdString() + " : " + obs_item->sensorLabel;
			m_ui->viewer_container->updateImageViewer(sensor_id, image);

			if(item->cloud() != nullptr)
				m_ui->viewer_container->updateCloudViewer(sensor_id, item->cloud(), viewer_text);

			else
			{
				T3DPointsProjectionParams projection_params;
				projection_params.MAKE_DENSE = false;
				projection_params.MAKE_ORGANIZED = true;

				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
				obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
				cloud->is_dense = false;
				m_ui->viewer_container->updateCloudViewer(sensor_id, cloud, viewer_text);

				item->setCloud(cloud);
			}

			if((m_calib_from_planes_gui != nullptr) && (m_calib_from_planes_gui->calibStatus() == CalibFromPlanesStatus::PLANES_EXTRACTED
			                                           || m_calib_from_planes_gui->calibStatus() == CalibFromPlanesStatus::PLANES_MATCHED))
				m_calib_from_planes_gui->publishPlanes(sensor_id, sync_obs_id);

			else if((m_calib_from_lines_gui != nullptr) && (m_calib_from_lines_gui->calibStatus() == CalibFromLinesStatus::LINES_EXTRACTED
			                                           || m_calib_from_lines_gui->calibStatus() == CalibFromLinesStatus::LINES_MATCHED))
				m_calib_from_lines_gui->publishLines(sensor_id, sync_obs_id);
		}

		//else set-item was clicked
		else
		{
			for(int i = 0; i < item->childCount(); i++)
			{
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(item->child(i)->getObservation());
				obs_item->getDescriptionAsText(update_stream);
				image = std::make_shared<mrpt::img::CImage>(obs_item->intensityImage);

				sensor_id = utils::findItemIndexIn(m_sync_model->getSensorLabels(), obs_item->sensorLabel);
				sync_obs_id = utils::findItemIndexIn(m_sync_model->getSyncIndices()[sensor_id], item->child(i)->getPriorIndex());
				viewer_text = (m_sync_model->data(index)).toString().toStdString() + " : " + obs_item->sensorLabel;
				update_stream << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n";
				m_ui->viewer_container->updateImageViewer(sensor_id, image);

				//for debugging
				//m_ui->viewer_container->updateText(std::to_string(viewer_id) + " " + std::to_string(sync_obs_id));

				if(item->child(i)->cloud() != nullptr)
				{
					m_ui->viewer_container->updateCloudViewer(sensor_id, item->child(i)->cloud(), viewer_text);
					m_ui->viewer_container->updateSetCloudViewer(item->child(i)->cloud(), obs_item->sensorLabel,
					                                             m_sync_model->getSensorPoses()[sensor_id],
					                                             m_sync_model->data(index).toString().toStdString());
				}

				else
				{
					T3DPointsProjectionParams projection_params;
					projection_params.MAKE_DENSE = false;
					projection_params.MAKE_ORGANIZED = true;

					pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
					obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
					cloud->is_dense = false;

					m_ui->viewer_container->updateCloudViewer(sensor_id, cloud, viewer_text);
					m_ui->viewer_container->updateSetCloudViewer(cloud, obs_item->sensorLabel,
					                                             m_sync_model->getSensorPoses()[sensor_id],
					                                             (m_sync_model->data(index)).toString().toStdString());
					item->child(i)->setCloud(cloud);
				}

				if((m_calib_from_planes_gui != nullptr) && (m_calib_from_planes_gui->calibStatus() == CalibFromPlanesStatus::PLANES_EXTRACTED
				                                            || m_calib_from_planes_gui->calibStatus() == CalibFromPlanesStatus::PLANES_MATCHED))
					m_calib_from_planes_gui->publishPlanes(sensor_id, sync_obs_id);


				else if((m_calib_from_lines_gui != nullptr) && (m_calib_from_lines_gui->calibStatus() == CalibFromLinesStatus::LINES_EXTRACTED
				                                           || m_calib_from_lines_gui->calibStatus() == CalibFromLinesStatus::LINES_MATCHED))
					m_calib_from_lines_gui->publishLines(sensor_id, sync_obs_id);
			}

			if((m_calib_from_planes_gui != nullptr) && (m_calib_from_planes_gui->calibStatus() == CalibFromPlanesStatus::PLANES_MATCHED))
				m_calib_from_planes_gui->publishCorrespPlanes(item->row());

			else if((m_calib_from_lines_gui != nullptr) && (m_calib_from_lines_gui->calibStatus() == CalibFromLinesStatus::LINES_MATCHED))
				m_calib_from_lines_gui->publishCorrespLines(item->row());
		}
    
		m_ui->observations_description_textbrowser->setText(QString::fromStdString(update_stream.str()));
	}
}

void CMainWindow::algosIndexChanged(int index)
{
	switch(index)
	{
	case 0:
	{
		// All the attempts to dynamically load/remove widgets that did not work.

		//m_config_widget.reset();

	   //QLayoutItem *config_widget_item = qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->takeAt(2);
		//if(config_widget_item)
		//qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->removeItem(config_widget_item);

		//qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->removeWidget(m_config_widget.get());

		//QLayoutItem *child;
		//while ((child = qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->takeAt(0)) != 0)
		    //delete child;

		//m_config_widget = nullptr;
		//delete qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->itemAt(3);

		m_calib_from_planes_config_widget.get()->hide();
		m_calib_from_lines_config_widget.get()->hide();
		m_calib_from_lines_gui = nullptr;
		m_calib_from_planes_gui = nullptr;
		m_ui->viewer_container->calib_from_lines = false;
		m_ui->viewer_container->calib_from_planes = false;
		m_ui->viewer_container->resetViewers(m_sync_model->getSensorLabels(), m_sync_model->getSensorPoses());
		break;
	}

	case 1:
	{
		m_calib_from_lines_gui = nullptr;
		m_calib_from_lines_config_widget.get()->hide();
		m_calib_from_planes_config_widget.get()->show();
		m_ui->viewer_container->calib_from_lines = false;
		m_ui->viewer_container->calib_from_planes = true;
		m_ui->viewer_container->resetViewers(m_sync_model->getSensorLabels(), m_sync_model->getSensorPoses());
		break;
	}

	case 2:
	{
		m_calib_from_planes_gui = nullptr;
		m_calib_from_planes_config_widget.get()->hide();
		m_calib_from_lines_config_widget.get()->show();
		m_ui->viewer_container->calib_from_planes = false;
		m_ui->viewer_container->calib_from_lines = true;
		m_ui->viewer_container->resetViewers(m_sync_model->getSensorLabels(), m_sync_model->getSensorPoses());

		//m_config_widget = std::make_shared<CCalibFromLinesConfig>(m_config_file);
		//qobject_cast<QVBoxLayout*>(m_ui->config_dockwidget_contents->layout())->insertWidget(3, m_config_widget.get());

		break;
	}
	}
}

void CMainWindow::runCalibFromPlanes(TCalibFromPlanesParams *params)
{
	switch(params->calib_status)
	{
	case CalibFromPlanesStatus::PCALIB_YET_TO_START:
	{
		if(m_sync_model != nullptr && (m_sync_model->getRootItem()->childCount() > 0))
		{
			m_calib_from_lines_gui = nullptr;
			m_calib_from_planes_gui = new CCalibFromPlanesGui(m_sync_model, params);
			m_calib_from_planes_gui->addTextObserver(m_ui->viewer_container);
			m_calib_from_planes_gui->addPlanesObserver(m_ui->viewer_container);
			m_calib_from_planes_gui->addCorrespPlanesObserver(m_ui->viewer_container);
			m_calib_from_planes_gui->extractPlanes();
			//std::thread thr(&CCalibFromPlanesGui::extractPlanes, m_calib_from_planes_gui);
			//thr.detach();
		}

		else
			m_ui->viewer_container->updateText("No grouped observations available!");

		break;
	}

	case CalibFromPlanesStatus::PLANES_EXTRACTED:
	{
		m_calib_from_planes_gui->matchPlanes();
		break;
	}

	case CalibFromPlanesStatus::PLANES_MATCHED:
	{
		m_calib_from_planes_gui->calibrate();
		break;
	}
	}
}

void CMainWindow::runCalibFromLines(TCalibFromLinesParams *params)
{
	switch(params->calib_status)
	{
	case CalibFromLinesStatus::LCALIB_YET_TO_START:
	{
		if(m_sync_model != nullptr && (m_sync_model->getRootItem()->childCount() > 0))
		{
			m_calib_from_planes_gui = nullptr;
			m_calib_from_lines_gui = new CCalibFromLinesGui(m_sync_model, params);
			m_calib_from_lines_gui->addTextObserver(m_ui->viewer_container);
			m_calib_from_lines_gui->addLinesObserver(m_ui->viewer_container);
			m_calib_from_lines_gui->addCorrespLinesObserver(m_ui->viewer_container);
			m_calib_from_lines_gui->extractLines();
		}

		else
			m_ui->viewer_container->updateText("No grouped observations available!");

		break;
	}

	case CalibFromLinesStatus::LINES_EXTRACTED:
	{
		m_calib_from_lines_gui->matchLines();
		break;
	}
	}
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
