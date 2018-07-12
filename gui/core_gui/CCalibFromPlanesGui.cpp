#include "CCalibFromPlanesGui.h"
#include <calib_solvers/CCalibFromPlanes.h>
#include <utils/CPlanes.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/math/types_math.h>
#include <mrpt/maps/PCL_adapters.h>
#include <mrpt/pbmap/PbMap.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>

using namespace mrpt::obs;

CCalibFromPlanesGui::CCalibFromPlanesGui(CObservationTreeModel *model, std::array<double, 6> init_calib, TPlaneMatchingParams params)
{
	m_model = model;
	m_init_calib = init_calib;
	m_params = params;
}

CCalibFromPlanesGui::~CCalibFromPlanesGui()
{
}

void CCalibFromPlanesGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CCalibFromPlanesGui::addPlanesObserver(CPlanesObserver *observer)
{
	m_planes_observers.push_back(observer);
}

void CCalibFromPlanesGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CCalibFromPlanesGui::publishPlaneCloud(const int &set_num, const int &cloud_num, const int &sensor_id)
{
	for(CPlanesObserver *observer : m_planes_observers)
	{
        observer->onReceivingPlaneCloud(sensor_id, m_extracted_plane_clouds_sets[set_num]);
	}
}

void CCalibFromPlanesGui::run()
{
	// For running all steps at a time
}

void CCalibFromPlanesGui::extractPlanes()
{
	publishText("****Running plane matching calibration algorithm****");

	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;
	root_item = m_model->getRootItem();

	T3DPointsProjectionParams params;
	params.MAKE_DENSE = false;
	params.MAKE_ORGANIZED = true;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	CCalibFromPlanes calib(2);

    TPlaneSegmentationParams seg_params;
    seg_params.normal_estimation_method = m_params.normal_estimation_method;
    seg_params.depth_dependent_smoothing = m_params.depth_dependent_smoothing;
    seg_params.max_depth_change_factor = m_params.max_depth_change_factor;
    seg_params.normal_smoothing_size = m_params.normal_smoothing_size;
    seg_params.angle_threshold = m_params.angle_threshold;
    seg_params.dist_threshold = m_params.dist_threshold;
    seg_params.min_inliers_frac = m_params.min_inliers_frac;
    seg_params.max_curvature = 0.1;

    calib.vvv_planes.resize(17); // [pair_id][sensor_id][plane_id]
	//for(int i = 0; i < root_item->childCount(); i++)
	// using only few observations for memory reasons
    for(int i = 0; i < 5; i++)
	{
		tree_item = root_item->child(i);

        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> extracted_plane_clouds;
		calib.vvv_planes[i] = std::vector< std::vector< CPlaneCHull > >(tree_item->childCount());
		for(int j = 0; j < tree_item->childCount(); j++)
		{
			publishText("**Extracting planes from #" + std::to_string(j) + " observation in observation set #" + std::to_string(i) + "**");

			obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(tree_item->child(j)->getObservation());
			obs_item->project3DPointsFromDepthImageInto(*cloud, params);

            // Let's try to visualize the point cloud in color
//            for (size_t i = 0; i < cloud->size(); ++i) {
//                cloud->points[i].rgb = obs_item->intensityImage.getAsFloat(i%640, i/640);
////                cloud->points[i].r = static_cast<uint8_t>(obs_item->intensityImage.get_unsafe(i%640, i/640, 0));
////                cloud->points[i].g = static_cast<uint8_t>(obs_item->intensityImage.get_unsafe(i%640, i/640, 1));
////                cloud->points[i].b = static_cast<uint8_t>(obs_item->intensityImage.get_unsafe(i%640, i/640, 2));
//            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr extracted_planes(new pcl::PointCloud<pcl::PointXYZRGBA>);
//            extractPlanes(cloud, extracted_planes);
//            extracted_plane_clouds.push_back(extracted_planes);

            size_t n_planes = segmentPlanes(cloud, seg_params, calib.vvv_planes[i][j]);
            //std::cout << n_planes << " extractPlanes\n";

            for(size_t ii = 0; ii < n_planes; ii++)
            {
                std::vector<int> & indices = calib.vvv_planes[i][j][ii].v_inliers;
                for(size_t k = 0; k < indices.size(); k++)
                {
                    extracted_planes->points.push_back(cloud->points[indices[k]]);
                }
            }
            extracted_plane_clouds.push_back(extracted_planes);
        }

		m_extracted_plane_clouds_sets.push_back(extracted_plane_clouds);

        // Check for potential plane matches
        calib.findPotentialMatches(calib.vvv_planes[i], i);
    }

    //calib.computeCalibration_rot(ExtrinsicCalib::m_init_calib);
}

void CCalibFromPlanesGui::proceed()
{
}
