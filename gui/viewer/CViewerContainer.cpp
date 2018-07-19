#include <viewer/CViewerContainer.h>
#include <ui_CViewerContainer.h>

#include <mrpt/pbmap/PbMap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

CViewerContainer::CViewerContainer(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CViewerContainer)
{
	m_ui->setupUi(this);

	std::string place_holder_str = "No input";

	m_viewers[0].reset(new pcl::visualization::PCLVisualizer("input1_viewer", false));
	m_viewers[0]->setBackgroundColor(0.3,0.3,0.3);
	m_viewers[0]->setShowFPS(false);
	m_viewers[0]->addText(place_holder_str, 10, 10, 1, 1, 1, "text");
	m_viewers[0]->resetCamera();

	m_ui->input1_viz->SetRenderWindow(m_viewers[0]->getRenderWindow());
	m_viewers[0]->setupInteractor(m_ui->input1_viz->GetInteractor(), m_ui->input1_viz->GetRenderWindow());
	m_ui->input1_viz->update();

	m_viewers[1].reset(new pcl::visualization::PCLVisualizer("input2_viewer", false));
	m_viewers[1]->setBackgroundColor(0.3,0.3,0.3);
	m_viewers[1]->addText(place_holder_str, 10, 10, 1, 1, 1, "text");
	m_viewers[1]->setShowFPS(false);
	m_viewers[1]->resetCamera();

	m_ui->input2_viz->SetRenderWindow(m_viewers[1]->getRenderWindow());
	m_viewers[1]->setupInteractor(m_ui->input2_viz->GetInteractor(), m_ui->input2_viz->GetRenderWindow());
	m_ui->input2_viz->update();

	m_viewers[2].reset(new pcl::visualization::PCLVisualizer("output_viewer", false));
	m_viewers[2]->setBackgroundColor(0.3,0.3,0.3);
	m_viewers[2]->setShowFPS(false);
	m_viewers[2]->addText(place_holder_str, 10, 10, 1, 1, 1, "text");
	m_viewers[2]->resetCamera();

	m_ui->result_viz->SetRenderWindow(m_viewers[2]->getRenderWindow());
	m_viewers[2]->setupInteractor(m_ui->result_viz->GetInteractor(), m_ui->result_viz->GetRenderWindow());
	m_ui->result_viz->update();
}

CViewerContainer::~CViewerContainer()
{
	delete m_ui;
}

void CViewerContainer::updateText(const std::string &text)
{
	m_ui->text_output->moveCursor(QTextCursor::End);
	m_ui->text_output->insertPlainText(QString::fromStdString(text) + QString("\n\n"));
}

bool CViewerContainer::viewerContainsCloud(const int &viewer_id, const std::string &id)
{
	pcl::visualization::CloudActorMapPtr cloud_actor_map;

	cloud_actor_map = m_viewers[viewer_id]->getCloudActorMap();
	return (cloud_actor_map->find(id) != cloud_actor_map->end ());
}

void CViewerContainer::updateCloudViewer(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> viewer_color_handler(cloud);

	m_viewers[viewer_id]->removeAllPointClouds();
	m_viewers[viewer_id]->removeAllShapes();
	m_viewers[viewer_id]->addPointCloud(cloud, viewer_color_handler, "cloud");
	m_viewers[viewer_id]->resetCamera();
	m_viewers[viewer_id]->updateText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::updateSetCloudViewer(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &sensor_label, const Eigen::Matrix4f &relative_transformation, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> viewer_color_handler(cloud);
	Eigen::Affine3f rT;
	rT.matrix() = relative_transformation;

	if(viewerContainsCloud(2, sensor_label))
	{
		m_viewers[2]->removePointCloud(sensor_label);
		m_viewers[2]->removeCoordinateSystem(sensor_label);
	}

	m_viewers[2]->addPointCloud(cloud, viewer_color_handler, sensor_label);
	m_viewers[2]->updatePointCloudPose(sensor_label, rT);
	m_viewers[2]->resetCamera();
	m_viewers[2]->updateText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[2]->addCoordinateSystem(0.3, rT, sensor_label);

	m_ui->result_viz->update();
}

void CViewerContainer::onReceivingPlanes(const int &viewer_id, const std::vector<CPlaneCHull> &planes)
{
	char plane_id[1024], polygon_id[1024];

	for(size_t i=0; i < planes.size(); i++)
	{
		cv::Mat img = cv::cvarrToMat((m_viewer_images[viewer_id]).getAs<IplImage>());
		size_t width = img.cols;

		sprintf (plane_id, "plane_%u", static_cast<unsigned>(i));
		sprintf(polygon_id, "polygon_%u", static_cast<unsigned>(i));

		pcl::PointXYZ pt1, pt2;
		pt1 = pcl::PointXYZ(planes[i].v3center[0], planes[i].v3center[1], planes[i].v3center[2]);
		pt2 = pcl::PointXYZ(planes[i].v3center[0] + (0.5f * planes[i].v3normal[0]),
		        planes[i].v3center[1] + (0.5f * planes[i].v3normal[1]),
		        planes[i].v3center[2] + (0.5f * planes[i].v3normal[2]));

		m_viewers[viewer_id]->addArrow(pt2, pt1, 0.5 * viz_colors::red[i%10] / 255, viz_colors::grn[i%10] / 255, viz_colors::blu[i%10] / 255, false, plane_id);
		m_viewers[viewer_id]->addPolygon<pcl::PointXYZRGBA>(planes[i].ConvexHullPtr, viz_colors::red[i%10], viz_colors::grn[i%10], viz_colors::blu[i%10], polygon_id);

		size_t indices_size = planes[i].v_hull_indices.size();
		for(size_t j = 0; j < indices_size; j++)
		{
			cv::line(img, cv::Point(planes[i].v_hull_indices[j]%width, planes[i].v_hull_indices[j]/width),
			         cv::Point(planes[i].v_hull_indices[(j+1)%indices_size]%width, planes[i].v_hull_indices[(j+1)%indices_size]/width),
			         cv::Scalar(viz_colors::blu[i%10], viz_colors::grn[i%10], viz_colors::red[i%10]), 3);
		}

		if(i == planes.size())
		{
			IplImage *iimg = new IplImage(img);
			mrpt::img::CImage cimg;
			cimg.setFromIplImage(iimg);
			updateImageViewer(viewer_id, cimg);
		}
	}

	m_viewers[viewer_id]->resetCamera();
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::updateImageViewer(const int &viewer_id, mrpt::img::CImage &image)
{
	m_viewer_images[viewer_id] = image;

	switch(viewer_id)
	{
	case 0:
	{
		m_ui->input1_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(image);
		m_ui->input1_tab_widget->insertTab(1, gl, "RGB");
		break;
	}

	case 1:
	{
		m_ui->input2_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(image);
		m_ui->input2_tab_widget->insertTab(1, gl, "RGB");
		break;
	}
	}
}

void CViewerContainer::onReceivingText(const std::string &msg)
{
	updateText(msg);
}
