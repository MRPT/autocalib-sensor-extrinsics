#include <viewer/CViewerContainer.h>
#include <ui_CViewerContainer.h>

// Define some colours to draw bolobs, patches, etc.
static const unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
static const unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
static const unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

//Q_DECLARE_METATYPE(QTextCursor)

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
//	qRegisterMetaType<QTextCursor>();
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
	m_viewers[viewer_id]->addPointCloud(cloud, viewer_color_handler, "cloud");
	m_viewers[viewer_id]->resetCamera();
	m_viewers[viewer_id]->updateText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[viewer_id]->addCoordinateSystem(0.3);

	m_ui->input1_viz->update();
	m_ui->input2_viz->update();
	m_ui->result_viz->update();
}

void CViewerContainer::updateSetCloudViewer(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &sensor_label, const std::string &text)
{
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> viewer_color_handler(cloud);

	if(viewerContainsCloud(2, sensor_label))
		m_viewers[2]->removePointCloud(sensor_label);

	m_viewers[2]->addPointCloud(cloud, viewer_color_handler, sensor_label);
	m_viewers[2]->resetCamera();
	m_viewers[2]->updateText(text, 10, 10, 1, 1, 1, "text");
	m_viewers[2]->addCoordinateSystem(0.3);

	m_ui->result_viz->update();
}

void CViewerContainer::onReceivingPlaneCloud(const int &sensor_id, const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &cloud)
{
	char name[1024];
	for(size_t i=0; i < cloud.size(); i++)
	{
		sprintf (name, "plane_%u", static_cast<unsigned>(i));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> viewer_color_handler(cloud[i], red[i%10], grn[i%10], blu[i%10]);
		m_viewers[sensor_id]->addPointCloud(cloud[i], viewer_color_handler, name);

//      mrpt::pbmap::Plane &plane_i = frame_->planes_.vPlanes[i];
//      sprintf (name, "normal_%u", static_cast<unsigned>(i));
//      pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//      pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//      pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
//      					plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
//                          plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
//      viz.addArrow(pt2, pt1, utils::nred[i%10], utils::ngrn[i%10], utils::nblu[i%10], false, name);

//      {
//      	sprintf(name, "n%u %s", static_cast<unsigned>(i), plane_i.label.c_str());
//          sprintf (name, "n%u %.1f %.2f", static_cast<unsigned>(i), plane_i.curvature*1000, plane_i.areaHull);
//          viz.addText3D(name, pt2, 0.1, utils::nred[i%10], utils::ngrn[i%10], utils::nblu[i%10], name);
//      }

//      sprintf (name, "approx_plane_%02d", int (i));
//      viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * utils::red[i%10], 0.5 * utils::grn[i%10], 0.5 * utils::blu[i%10], name);

		m_viewers[sensor_id]->resetCamera();
		m_viewers[sensor_id]->addCoordinateSystem(0.3);

		m_ui->input1_viz->update();
		m_ui->input2_viz->update();
		m_ui->result_viz->update();
	}
}

void CViewerContainer::updateImageViewer(const int &viewer_id, mrpt::img::CImage &image)
{
	switch(viewer_id)
	{
	case 1:
	{
		m_ui->input1_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView_fast(image);
		m_ui->input1_tab_widget->insertTab(1, gl, "RGB");
		break;
	}

	case 2:
	{
		m_ui->input2_tab_widget->removeTab(1);
		mrpt::gui::CQtGlCanvasBase *gl = new mrpt::gui::CQtGlCanvasBase();
		gl->mainViewport()->setImageView_fast(image);
		m_ui->input2_tab_widget->insertTab(1, gl, "RGB");
		break;
	}
	}
}

void CViewerContainer::onReceivingText(const std::string &msg)
{
	updateText(msg);
}
