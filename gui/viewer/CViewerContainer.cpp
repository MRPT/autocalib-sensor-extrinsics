#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"

CViewerContainer::CViewerContainer(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CViewerContainer)
{
	m_ui->setupUi(this);

	m_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

	m_input1_viewer.reset(new pcl::visualization::PCLVisualizer("input1_viewer", false));
	m_input1_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_input1_viewer->addPointCloud(m_cloud, "cloud");
	m_input1_viewer->resetCamera();

	m_ui->input1_viz->SetRenderWindow(m_input1_viewer->getRenderWindow());
	m_input1_viewer->setupInteractor(m_ui->input1_viz->GetInteractor(), m_ui->input1_viz->GetRenderWindow());
	m_ui->input1_viz->update();

	m_input2_viewer.reset(new pcl::visualization::PCLVisualizer("input2_viewer", false));
	m_input2_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_input2_viewer->addPointCloud(m_cloud, "cloud");
	m_input2_viewer->resetCamera();

	m_ui->input2_viz->SetRenderWindow(m_input2_viewer->getRenderWindow());
	m_input2_viewer->setupInteractor(m_ui->input2_viz->GetInteractor(), m_ui->input2_viz->GetRenderWindow());
	m_ui->input2_viz->update();

	m_output_viewer.reset(new pcl::visualization::PCLVisualizer("output_viewer", false));
	m_output_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_output_viewer->addPointCloud(m_cloud, "cloud");
	m_output_viewer->resetCamera();

	m_ui->result_viz->SetRenderWindow(m_output_viewer->getRenderWindow());
	m_output_viewer->setupInteractor(m_ui->result_viz->GetInteractor(), m_ui->result_viz->GetRenderWindow());
	m_ui->result_viz->update();
}

CViewerContainer::~CViewerContainer()
{
	delete m_ui;
}

void CViewerContainer::changeOutputText(const QString &text)
{
	m_ui->text_output->setText(text);
}

void CViewerContainer::changeViewerPointCloud(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	switch(viewer_id)
	{
		case 1:
		{
			m_input1_viewer->updatePointCloud(cloud, "cloud");
			m_input1_viewer->addCoordinateSystem(0.3);
			m_input1_viewer->resetCamera();
			m_ui->input1_viz->update();
		}
		break;

		case 2:
		{
			m_input2_viewer->updatePointCloud(cloud, "cloud");
			m_input2_viewer->addCoordinateSystem(0.3);
			m_input2_viewer->resetCamera();
			m_ui->input2_viz->update();
		}
		break;

		case 3:
		{
			m_output_viewer->updatePointCloud(cloud, "cloud");
			m_output_viewer->addCoordinateSystem(0.3);
			m_output_viewer->resetCamera();
			m_ui->result_viz->update();
		}
		break;
	}
}
