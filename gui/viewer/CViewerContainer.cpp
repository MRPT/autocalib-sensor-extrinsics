#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"

CViewerContainer::CViewerContainer(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CViewerContainer)
{
	m_ui->setupUi(this);

	// Create and initialize a place holder point cloud
	m_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	m_cloud->points.resize(200);

	for(size_t i = 0; i < m_cloud->points.size(); i++)
	{
		m_cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		m_cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		m_cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);

		m_cloud->points[i].r = 128;
		m_cloud->points[i].g = 128;
		m_cloud->points[i].b = 128;
	}

	//Initialize vtkwidgets with place holder point cloud

	m_input1_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	m_ui->input1_viz->SetRenderWindow(m_input1_viewer->getRenderWindow());
	m_input1_viewer->setupInteractor(m_ui->input1_viz->GetInteractor(), m_ui->input1_viz->GetRenderWindow());
	m_ui->input1_viz->update();

	m_input1_viewer->addPointCloud(m_cloud, "cloud");
	m_input1_viewer->resetCamera();
	m_ui->input1_viz->update();


	m_input2_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	m_ui->input2_viz->SetRenderWindow(m_input2_viewer->getRenderWindow());
	m_input2_viewer->setupInteractor(m_ui->input2_viz->GetInteractor(), m_ui->input2_viz->GetRenderWindow());
	m_ui->input2_viz->update();

	m_input2_viewer->addPointCloud(m_cloud, "cloud");
	m_input2_viewer->resetCamera();
	m_ui->input2_viz->update();


	m_output_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));

	m_ui->output_viz->SetRenderWindow(m_output_viewer->getRenderWindow());
	m_output_viewer->setupInteractor(m_ui->output_viz->GetInteractor(), m_ui->output_viz->GetRenderWindow());
	m_ui->output_viz->update();

	m_output_viewer->addPointCloud(m_cloud, "cloud");
	m_output_viewer->resetCamera();
	m_ui->output_viz->update();

}

CViewerContainer::~CViewerContainer()
{
	delete m_ui;
}

void CViewerContainer::changeOutputText(const QString &text)
{
	m_ui->text_output->setText(text);
}
