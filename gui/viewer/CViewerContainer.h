#pragma once

#include <utils/CTextObserver.h>
#include <utils/CPlanesObserver.h>

#include <QWidget>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>

namespace Ui {
class CViewerContainer;
}

class CViewerContainer : public QWidget, public CTextObserver, public CPlanesObserver
{
	Q_OBJECT

public:
	explicit CViewerContainer(QWidget *parent = 0);
	~CViewerContainer();
	void updateText(const std::string &);
	void updateViewer(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::string &text);
	void addPlanes(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
	void updateCalibConfig(const int &calib_algo_id);
	bool viewerContainsCloud(const int &viewer_id, const std::string &id);
	virtual void onReceivingText(const std::string &msg);
	virtual void onReceivingPlaneCloud(const int &obs_type, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

private:
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_input1_viewer;
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_input2_viewer;
	std::shared_ptr<pcl::visualization::PCLVisualizer> m_output_viewer;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_viewer_cloud;
	std::shared_ptr<std::string> m_viewer_text;

	Ui::CViewerContainer *m_ui;
};
