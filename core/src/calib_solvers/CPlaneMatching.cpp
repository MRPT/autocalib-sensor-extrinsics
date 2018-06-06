#include <calib_solvers/CPlaneMatching.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/math/types_math.h>
#include <mrpt/maps/PCL_adapters.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/time.h>

using namespace mrpt::obs;
using namespace mrpt::maps;

CPlaneMatching::CPlaneMatching(CObservationTreeModel *model/*, Ui::CMainWindow *main_ui*/, CPlaneMatchingParams params)
{
	m_model = model;
	//m_main_ui = main_ui;
	m_params = params;
}

CPlaneMatching::~CPlaneMatching()
{
}

void CPlaneMatching::run()
{
	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::stringstream stream;
	T3DPointsProjectionParams projection_params;
	projection_params.MAKE_DENSE = true;

	root_item = m_model->getRootItem();

	for(int i = 0; i < m_model->rowCount(); i++)
	{
		tree_item = root_item->child(i);
		for(int j = 0; j < tree_item->childCount(); j++)
		{
			stream << "**Extracting planes from #" << i << " set, #" << j << " observation**\n\n";
			//m_main_ui->viewer_container->changeOutputText(QString::fromStdString(stream.str()));

			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
			obs_item->load();
			obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
			obs_item->unload();

			detectPlanes(cloud, stream);
		}
	}
}

void CPlaneMatching::detectPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::stringstream &stream)
{
	unsigned min_inliers = m_params.min_inliers_frac * cloud->size();

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
	normal_estimation.setMaxDepthChangeFactor(0.02f);
	normal_estimation.setNormalSmoothingSize(10.0f);
	normal_estimation.setDepthDependentSmoothing(true);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> multi_plane_segmentation;
	multi_plane_segmentation.setMinInliers(min_inliers);
	multi_plane_segmentation.setAngularThreshold(m_params.angle_threshold);
	multi_plane_segmentation.setDistanceThreshold(m_params.dist_threshold);

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setInputCloud(cloud);
	normal_estimation.compute(*normal_cloud);

	double plane_extract_start = pcl::getTime();
	multi_plane_segmentation.setInputNormals(normal_cloud);
	multi_plane_segmentation.setInputCloud(cloud);

	std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ>>> regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;

	multi_plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
	double plane_extract_end = pcl::getTime();

	stream << regions.size() << " plane(s) detected\n" << "Time elapsed: " << double(plane_extract_end - plane_extract_start) << std::endl;
}

void CPlaneMatching::proceed()
{
}
