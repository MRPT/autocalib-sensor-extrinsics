#pragma once

#include <interfaces/CTextObserver.h>
#include <interfaces/CPlanesObserver.h>
#include <interfaces/CCorrespPlanesObserver.h>
#include <CPlanes.h>
#include <CUtils.h>

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

class CViewerContainer : public QWidget, public CTextObserver, public CPlanesObserver, public CCorrespPlanesObserver
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
	 * \param image the image to be displayed.
	 */
	void updateImageViewer(const int &viewer_id, mrpt::img::CImage &image);

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
	 * \param corresp_planes the matched planes from each sensor that are to be visualized.
	 */
	virtual void onReceivingCorrespPlanes(const std::vector<std::vector<CPlaneCHull>> &corresp_planes);

private:

	/** An array of three PCLVisualizer objects, each linked to one viewer widget. */
	std::array<std::shared_ptr<pcl::visualization::PCLVisualizer>, 3> m_viewers;

	/** An array of images that are a copy of the images displayed in the image viewers. */
	std::array<mrpt::img::CImage, 2> m_viewer_images;

	Ui::CViewerContainer *m_ui;

};
