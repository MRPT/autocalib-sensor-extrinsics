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

	connect(m_ui->input1_cbox, SIGNAL(activated(int)), this, SLOT(sensorIndexChanged(int)));
	connect(m_ui->input2_cbox, SIGNAL(activated(int)), this, SLOT(sensorIndexChanged(int)));

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

void CViewerContainer::updateSensorsList(const std::vector<std::string> &sensor_labels)
{
	m_sensor_labels = sensor_labels;

	for(int i = 0; i < sensor_labels.size(); i++)
	{
		m_ui->input1_cbox->insertItem(i, QString::fromStdString(sensor_labels[i]));
		m_ui->input2_cbox->insertItem(i, QString::fromStdString(sensor_labels[i]));
	}

	m_ui->input2_cbox->setCurrentIndex(1);
}

void CViewerContainer::sensorIndexChanged(const int &sensor_id)
{
	QComboBox *cbox = (QComboBox*)sender();

	if(utils::findItemIndexIn(m_vsensor_ids, sensor_id) != -1)
	{
		updateText("Error! Sensor already exists. Choose a different sensor.");
		if(cbox->accessibleName() == QString("input1"))
			cbox->setCurrentIndex(m_vsensor_ids[0]);
		else
			cbox->setCurrentIndex(m_vsensor_ids[1]);
	}

	else
	{
		if(cbox->accessibleName() == QString("input1"))
			m_vsensor_ids[0] = sensor_id;
		else
			m_vsensor_ids[1] = sensor_id;
	}
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
	m_viewers[viewer_id]->addText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::updateCloudViewers(const int &sensor_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> viewer_color_handler(cloud);

	int viewer_id = utils::findItemIndexIn(m_vsensor_ids, sensor_id);

	if(viewer_id == -1)
	{
		m_vsensor_ids[1] = sensor_id;
		viewer_id = 1;
		m_ui->input2_cbox->setCurrentIndex(sensor_id);
	}

	m_viewers[viewer_id]->removeAllPointClouds();
	m_viewers[viewer_id]->removeAllShapes();
	m_viewers[viewer_id]->addPointCloud(cloud, viewer_color_handler, "cloud");
	m_viewers[viewer_id]->resetCamera();
	m_viewers[viewer_id]->addText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::onReceivingPlanes(const int &viewer_id, const std::vector<CPlaneCHull> &planes)
{
	char normal_id[1024], polygon_id[1024];

	cv::Mat img = cv::cvarrToMat(m_viewer_images[viewer_id]->getAs<IplImage>());
	size_t width = img.cols;

	for(size_t i=0; i < planes.size(); i++)
	{
		sprintf(normal_id, "plane_normal_%u", static_cast<unsigned>(i));
		sprintf(polygon_id, "plane_polygon_%u", static_cast<unsigned>(i));

		pcl::PointXYZ pt1, pt2;
		pt1 = pcl::PointXYZ(planes[i].v3center[0], planes[i].v3center[1], planes[i].v3center[2]);
		pt2 = pcl::PointXYZ(planes[i].v3center[0] + (0.5f * planes[i].v3normal[0]),
		        planes[i].v3center[1] + (0.5f * planes[i].v3normal[1]),
		        planes[i].v3center[2] + (0.5f * planes[i].v3normal[2]));

		m_viewers[viewer_id]->addArrow(pt2, pt1, 0.5 * utils::colors::red[i%10] / 255, utils::colors::grn[i%10] / 255, utils::colors::blu[i%10] / 255, false, normal_id);
		m_viewers[viewer_id]->addPolygon<pcl::PointXYZRGBA>(planes[i].ConvexHullPtr, utils::colors::red[i%10], utils::colors::grn[i%10], utils::colors::blu[i%10], polygon_id);

		size_t indices_size = planes[i].v_hull_indices.size();
		for(size_t j = 0; j < indices_size; j++)
		{
			cv::line(img, cv::Point(planes[i].v_hull_indices[j]%width, planes[i].v_hull_indices[j]/width),
			         cv::Point(planes[i].v_hull_indices[(j+1)%indices_size]%width, planes[i].v_hull_indices[(j+1)%indices_size]/width),
			        cv::Scalar(utils::colors::blu[i%10], utils::colors::grn[i%10], utils::colors::red[i%10]), 3);
		}
	}

	IplImage *iimg = new IplImage(img);
	mrpt::img::CImage::Ptr cimg(new mrpt::img::CImage(iimg));
	updateImageViewer(viewer_id, cimg, true);

	m_viewers[viewer_id]->resetCamera();
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::updateSetCloudViewer(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &sensor_label, const Eigen::Matrix4f &sensor_pose, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> viewer_color_handler(cloud);
	Eigen::Affine3f rt;
	rt.matrix() = sensor_pose;

	if(viewerContainsCloud(2, sensor_label))
	{
		m_viewers[2]->removePointCloud(sensor_label);
		m_viewers[2]->removeCoordinateSystem(sensor_label);
	}

	m_viewers[2]->addPointCloud(cloud, viewer_color_handler, sensor_label);
	m_viewers[2]->updatePointCloudPose(sensor_label, rt);
	m_viewers[2]->resetCamera();
	m_viewers[2]->updateText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[2]->addCoordinateSystem(0.3, rt, sensor_label);

	m_ui->result_viz->update();
}

void CViewerContainer::onReceivingCorrespPlanes(std::map<int,std::map<int,std::vector<std::array<CPlaneCHull,2>>>> &corresp_planes, const std::vector<Eigen::Matrix4f> &sensor_poses)
{
	m_viewers[2]->removeAllShapes();
	char normal_id[1024], polygon_id[1024];
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_polygon(new pcl::PointCloud<pcl::PointXYZRGBA>);
	Eigen::Affine3f rt;

	for(int i = 0; i < corresp_planes[m_vsensor_ids[0]][m_vsensor_ids[1]].size(); i++)
	{
		std::array<CPlaneCHull,2> planes_pair = corresp_planes[m_vsensor_ids[0]][m_vsensor_ids[1]][i];
		for(int j = 0; j < 2; j++)
		{
			sprintf(normal_id, "%u_plane_normal_%u", static_cast<unsigned>(i), static_cast<unsigned>(j));
			sprintf(polygon_id, "%u_plane_polygon_%u", static_cast<unsigned>(i), static_cast<unsigned>(j));

			pcl::PointXYZ pt1, pt2;
			pt1 = pcl::PointXYZ(planes_pair[j].v3center[0], planes_pair[j].v3center[1], planes_pair[j].v3center[2]);
			pt2 = pcl::PointXYZ(planes_pair[j].v3center[0] + (0.5f * planes_pair[j].v3normal[0]),
			        planes_pair[j].v3center[1] + (0.5f * planes_pair[j].v3normal[1]),
			        planes_pair[j].v3center[2] + (0.5f * planes_pair[j].v3normal[2]));

			rt.matrix() = sensor_poses[m_vsensor_ids[j]];
			pcl::transformPointCloud(*planes_pair[j].ConvexHullPtr, *transformed_polygon, rt);
			utils::transformPoint(rt, pt1);
			utils::transformPoint(rt, pt2);

			m_viewers[2]->addArrow(pt2, pt1, utils::colors::red[i%10] / 255, utils::colors::grn[i%10] / 255, utils::colors::blu[i%10] / 255, false, normal_id);
			m_viewers[2]->addPolygon<pcl::PointXYZRGBA>(transformed_polygon, utils::colors::red[i%10], utils::colors::grn[i%10], utils::colors::blu[i%10], polygon_id);
		}
	}

	m_viewers[2]->resetCamera();
	//m_viewers[2]->addCoordinateSystem(0.3);
	m_ui->result_viz->update();
}

void CViewerContainer::updateImageViewer(const int &viewer_id, mrpt::img::CImage::Ptr &image, const bool &draw)
{
	if(!draw)
		m_viewer_images[viewer_id] = image;

	switch(viewer_id)
	{
	case 0:
	{
		m_ui->input1_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(*image);
		m_ui->input1_tab_widget->insertTab(1, gl, "RGB");
		break;
	}

	case 1:
	{
		m_ui->input2_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(*image);
		m_ui->input2_tab_widget->insertTab(1, gl, "RGB");
		break;
	}
	}
}

void CViewerContainer::updateImageViewers(const int &sensor_id, mrpt::img::CImage::Ptr &image, const bool &draw)
{
	int viewer_id = utils::findItemIndexIn(m_vsensor_ids, sensor_id);

	if(!draw)
		m_viewer_images[viewer_id] = image;

	switch(viewer_id)
	{
	case 0:
	{
		m_ui->input1_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(*image);
		m_ui->input1_tab_widget->insertTab(1, gl, "RGB");
		break;
	}

	case 1:
	{
		m_ui->input2_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView(*image);
		m_ui->input2_tab_widget->insertTab(1, gl, "RGB");
		break;
	}
	}
}

void CViewerContainer::onReceivingLines(const int &viewer_id, const std::vector<CLine> &lines)
{
	char line_id[1024];
	cv::Vec2i pt1, pt2;
	pcl::PointXYZ ppt1, ppt2;

	cv::Mat img = cv::cvarrToMat((m_viewer_images[viewer_id])->getAs<IplImage>());

	for(int i = 0; i < lines.size(); i++)
	{
		CLine l = lines[i];
		pt1 = cv::Vec2i(l.end_points[0][0], l.end_points[0][1]);
		pt2 = cv::Vec2i(l.end_points[1][0], l.end_points[1][1]);
		cv::line(img, pt1, pt2, cv::Scalar(utils::colors::blu[i%10], utils::colors::grn[i%10], utils::colors::red[i%10]), 3);

		sprintf(line_id, "line_%u", static_cast<unsigned>(i));

		ppt1.x = l.end_points3D[0][0];
		ppt1.y = l.end_points3D[0][1];
		ppt1.z = l.end_points3D[0][2];
		ppt2.x = l.end_points3D[1][0];
		ppt2.y = l.end_points3D[1][1];
		ppt2.z = l.end_points3D[1][2];

		m_viewers[viewer_id]->addLine(ppt2, ppt1, utils::colors::red[i%10] / 255, utils::colors::grn[i%10] / 255, utils::colors::blu[i%10] / 255, line_id);
	}

	IplImage *iimg = new IplImage(img);
	mrpt::img::CImage::Ptr cimg(new mrpt::img::CImage(iimg));
	updateImageViewer(viewer_id, cimg, true);

	m_viewers[viewer_id]->resetCamera();
	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
}

void CViewerContainer::onReceivingCorrespLines(std::map<int,std::map<int,std::vector<std::array<CLine,2>>>> &corresp_lines, const std::vector<Eigen::Matrix4f> &sensor_poses)
{
	m_viewers[2]->removeAllShapes();
	char line_id[1024];
	Eigen::Affine3f rt;

	for(int i = 0; i < corresp_lines[m_vsensor_ids[0]][m_vsensor_ids[1]].size(); i++)
	{
		std::array<CLine,2> lines_pair = corresp_lines[m_vsensor_ids[0]][m_vsensor_ids[1]][i];
		for(int j = 0; j < 2; j++)
		{
			sprintf(line_id, "%u_line_%u", static_cast<unsigned>(i), static_cast<unsigned>(j));

			pcl::PointXYZ pt1, pt2;
			pt1 = pcl::PointXYZ(lines_pair[j].end_points3D[0][0], lines_pair[j].end_points3D[0][1], lines_pair[j].end_points3D[0][2]);
			pt2 = pcl::PointXYZ(lines_pair[j].end_points3D[1][0], lines_pair[j].end_points3D[1][1], lines_pair[j].end_points3D[1][2]);

			rt.matrix() = sensor_poses[m_vsensor_ids[j]];
			utils::transformPoint(rt, pt1);
			utils::transformPoint(rt, pt2);

			m_viewers[2]->addLine(pt2, pt1, utils::colors::red[i%10] / 255, utils::colors::grn[i%10] / 255, utils::colors::blu[i%10] / 255, line_id);
		}
	}

	m_viewers[2]->resetCamera();
	//m_viewers[2]->addCoordinateSystem(0.3);
	m_ui->result_viz->update();
}

void CViewerContainer::onReceivingText(const std::string &msg)
{
	updateText(msg);
}
