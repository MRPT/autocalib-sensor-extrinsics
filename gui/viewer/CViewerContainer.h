#pragma once

//#include <config/CPlaneMatchingConfig.h>

#include <QWidget>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

namespace Ui {
class CViewerContainer;
}

class CViewerContainer : public QWidget
{
	Q_OBJECT

public:
	explicit CViewerContainer(QWidget *parent = 0);
	~CViewerContainer();

	void changeOutputText(const QString &);
	void updateViewer(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &text);
	void updateCalibConfig(const int &calib_algo_id);
	Ui::CViewerContainer *getViewerPointer();
	//CPlaneMatchingParams getCalibConfigParams();

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_input1_viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_input2_viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_output_viewer;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_viewer_cloud;
	std::shared_ptr<std::string> m_viewer_text;

	Ui::CViewerContainer *m_ui;
};
