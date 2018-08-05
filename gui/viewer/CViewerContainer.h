#pragma once

#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>
#include <interfaces/CCorrespPlanesObserver.h>
#include <interfaces/CLinesObserver.h>
#include <CPlane.h>
#include <Utils.h>

#include <QWidget>

#include <mrpt/gui/CQtGlCanvasBase.h>
#include <mrpt/img/CImage.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <opencv2/core/core.hpp>

namespace Ui {
class CViewerContainer;
}

class CViewerContainer : public QWidget, public CTextObserver, public CPlanesObserver, public CCorrespPlanesObserver, public CLinesObserver
{
	Q_OBJECT

public:
	explicit CViewerContainer(QWidget *parent = 0);
	~CViewerContainer();

	/** Update the text browswer with received text message. */
	void updateText(const std::string &);

	/** Update cloud visualizer with a point cloud.
	 * \param viewer_id the id of the visualizer.
	 * \param cloud the input cloud.
	 * \param text to be displayed in the visualizer.
	 */
	void updateCloudViewer(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &text);

	/**
	 * Update the result viewer with all the observations in a set, overlapped.
	 * \param cloud the input cloud.
	 * \param sensor_label the label of the sensor the cloud belongs to.
	 * \param sensor_pose the relative pose of the sensor wrt the first sensor.
	 * \param text to be displayed in the visualizer.
	 */
	void updateSetCloudViewer(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &sensor_label, const Eigen::Matrix4f &sensor_pose, const std::string &text);

	/** Update the image viewer with an image.
	 * \param viewer_id the id of the image viewer to be updated.
	 * \param image pointer to the image to be displayed.
	 * \param draw indicates whether the image to be displayed is drawn with shapes.
	 */
	void updateImageViewer(const int &viewer_id, mrpt::img::CImage::Ptr &image, const bool &draw = 0);

	void updateCalibConfig(const int &calib_algo_id);
	bool viewerContainsCloud(const int &viewer_id, const std::string &id);
	virtual void onReceivingText(const std::string &msg);

	/** Updates the viewer with the received planes.
	 * \param viewer_id the id of the visualizer.
	 * \param planes the vector of planes to be added to the visualizer.
	 */
	virtual void onReceivingPlanes(const int &viewer_id, const std::vector<CPlaneCHull> &planes);

	/**
	 * \brief Updates the middle viewer with the matched planes.
	 * \param corresp_planes the set of correspondinging planes between each sensor pair in a set that are to be visualized.
	 */
	virtual void onReceivingCorrespPlanes(std::map<int,std::map<int,std::vector<std::array<CPlaneCHull,2>>>> &corresp_planes, const std::vector<Eigen::Matrix4f> &sensor_poses);

	/**
	 * \brief Updates the image viewer with the received line segments.
	 * \param viewer_id the id of the image viewer.
	 * \param lines the vector of line segments to be drawn.
	 */
	virtual void onReceivingLines(const int &viewer_id, const std::vector<CLine> &lines);

private:

	/** An array of three PCLVisualizer objects, each linked to one viewer widget. */
	std::array<std::shared_ptr<pcl::visualization::PCLVisualizer>, 3> m_viewers;

	/** An array of image pointers that point to the images displayed in the image viewers. */
	std::array<mrpt::img::CImage::Ptr, 2> m_viewer_images;

	/** A copy of the current set of corresponding planes to be displayed in the main viewer. */
	std::map<int,std::map<int,std::vector<std::array<int,4>>>> m_current_corresp_planes;

	/** The ids of the sensor whose observations are to be visualized in viewers 1 and 2 repsepctively.
	 * Only the observations of two sensors can be visualized in the viewers at a time. */
	std::vector<int> m_vsensor_ids = {0,1};

	Ui::CViewerContainer *m_ui;
};
