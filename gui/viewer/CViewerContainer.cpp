#include <viewer/CViewerContainer.h>
#include <ui_CViewerContainer.h>

CViewerContainer::CViewerContainer(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CViewerContainer)
{
	m_ui->setupUi(this);

	m_viewer_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	m_viewer_text.reset(new std::string("No input"));

	m_input1_viewer.reset(new pcl::visualization::PCLVisualizer("input1_viewer", false));
	m_input1_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_input1_viewer->addPointCloud(m_viewer_cloud, "cloud");
	m_input1_viewer->setShowFPS(false);
	m_input1_viewer->addText(*m_viewer_text, 10, 10, 1, 1, 1, "text");
	m_input1_viewer->resetCamera();

	m_ui->input1_viz->SetRenderWindow(m_input1_viewer->getRenderWindow());
	m_input1_viewer->setupInteractor(m_ui->input1_viz->GetInteractor(), m_ui->input1_viz->GetRenderWindow());
	m_ui->input1_viz->update();

	m_input2_viewer.reset(new pcl::visualization::PCLVisualizer("input2_viewer", false));
	m_input2_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_input2_viewer->addPointCloud(m_viewer_cloud, "cloud");
	m_input2_viewer->addText(*m_viewer_text, 10, 10, 1, 1, 1, "text");
	m_input2_viewer->setShowFPS(false);
	m_input2_viewer->resetCamera();

	m_ui->input2_viz->SetRenderWindow(m_input2_viewer->getRenderWindow());
	m_input2_viewer->setupInteractor(m_ui->input2_viz->GetInteractor(), m_ui->input2_viz->GetRenderWindow());
	m_ui->input2_viz->update();

	m_output_viewer.reset(new pcl::visualization::PCLVisualizer("output_viewer", false));
	m_output_viewer->setBackgroundColor(0.3,0.3,0.3);
	m_output_viewer->addPointCloud(m_viewer_cloud, "cloud");
	m_output_viewer->setShowFPS(false);
	m_output_viewer->addText(*m_viewer_text, 10, 10, 1, 1, 1, "text");
	m_output_viewer->resetCamera();

	m_ui->result_viz->SetRenderWindow(m_output_viewer->getRenderWindow());
	m_output_viewer->setupInteractor(m_ui->result_viz->GetInteractor(), m_ui->result_viz->GetRenderWindow());
	m_ui->result_viz->update();
}

CViewerContainer::~CViewerContainer()
{
	delete m_ui;
}

void CViewerContainer::onEvent(const std::string &msg)
{
	updateText(msg);
}

void CViewerContainer::onCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
	updateViewer(3, cloud, "Segmented Planes");
}

void CViewerContainer::updateText(const std::string &text)
{
	m_ui->text_output->moveCursor(QTextCursor::End);
	m_ui->text_output->insertPlainText(QString::fromStdString(text) + QString("\n\n"));
}

void CViewerContainer::updateViewer(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> viewer_color_handler(cloud);

	if(!(cloud->empty()))
	{
		switch(viewer_id)
		{
			case 1:
			{
				m_input1_viewer->updatePointCloud(cloud, viewer_color_handler, "cloud");
				m_input1_viewer->resetCamera();
				m_input1_viewer->updateText(text, 10, 10, 1, 1, 1, "text");
				m_input1_viewer->addCoordinateSystem(0.3);
				m_ui->input1_viz->update();
			}
			break;

			case 2:
			{
				m_input2_viewer->updatePointCloud(cloud, viewer_color_handler, "cloud");
				m_input2_viewer->resetCamera();
				m_input2_viewer->updateText(text, 10, 10, 1, 1, 1, "text");
				m_input2_viewer->addCoordinateSystem(0.3);
				m_ui->input2_viz->update();
			}
			break;

			case 3:
			{
				m_output_viewer->updatePointCloud(cloud, viewer_color_handler, "cloud");
				m_output_viewer->resetCamera();
				m_output_viewer->updateText(text, 10, 10, 1, 1, 1, "text");
				m_output_viewer->addCoordinateSystem(0.3);
				m_ui->result_viz->update();
			}
			break;
		}
	}
}
