#pragma once

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
	void changeViewerPointCloud(const int &viewer_id, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_input1_viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_input2_viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_output_viewer;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;

	Ui::CViewerContainer *m_ui;
};
