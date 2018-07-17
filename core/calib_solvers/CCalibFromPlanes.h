/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include "CExtrinsicCalib.h"
#include <CPlanes.h>
//#include <mrpt/pbmap/PbMap.h>
//#include <mrpt/pbmap/Miscellaneous.h>
#include <map>

struct TPlaneSegmentationParams
{
	//params for integral normal estimation
	int normal_estimation_method;
	bool depth_dependent_smoothing;
	double max_depth_change_factor;
	double normal_smoothing_size;

	//params for organized multiplane segmentation
	double angle_threshold;
	double dist_threshold;
	double min_inliers_frac;
	double max_curvature;
};


/*! Exploit 3D plane observations from set a of sensors to perform extrinsic calibration.
 *  Plane correspondences are analogous to the control points used to create panoramic images with a regular camera).
 *  It allows to estimate the extrinsic calibration between RGB-D sensors like Asus XPL.
 *
 */
//template <int num_sensors, typename Scalar = double>
class CCalibFromPlanes : public CExtrinsicCalib//<num_sensors, Scalar>
{
  public:

	/*! The back-projected clouds from the observations, indexes : [sensor_id][obs_id] */
	std::vector<std::vector< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > > vv_clouds;

    /*! The segmented planes, the vector indexes to access them are [sensor_id][obs_id][plane_id] */
	std::vector< std::vector< std::vector< CPlaneCHull > > > vvv_planes;

    /*! The plane correspondences between the different sensors. The map indexes correspond to the sensor IDs,
     * each matrix as as many rows as potential plane correspondences, with 4 columns for: obs_id1, plane_id1, obs_id2, plane_id2. */
    std::map<size_t, std::map<size_t, std::vector< std::array<size_t,4> > > > mmv_plane_corresp;

    /*! Covariance matrices */
    std::vector< Eigen::Matrix<Scalar,3,3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar,3,3> > > covariance_rot;
    std::vector< Eigen::Matrix<Scalar,3,3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar,3,3> > > m_covariance_trans;

    /*! Constructor */
	CCalibFromPlanes(size_t n_sensors = 2);

    /*! Destructor */
	virtual ~CCalibFromPlanes(){}

	void segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & cloud, const TPlaneSegmentationParams & params, std::vector<CPlaneCHull> & planes);

    /** Search for potentail plane Matches.
        \param plane_obs  */
	void findPotentialMatches(const std::vector< std::vector< CPlaneCHull > > & plane_obs, size_t obs_id);

    /** Calculate the residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    //virtual Scalar computeCalibResidual(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);
//    virtual Scalar computeCalibResidual(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

    /** Calculate the angular residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    virtual Scalar computeCalibResidual_rot(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

//    /** Calculate the translational residual error of the correspondences.
//        \param sensor_poses relative poses of the sensors
//        \return the residual */
//    virtual Scalar computeCalibResidual_trans(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

//    /** Compute Calibration.
//        \param sensor_poses initial calibration
//        \return the residual */
//    virtual Scalar computeCalibration(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

    /** Compute Calibration (only rotation).
        \param sensor_poses initial calibration
        \return the residual */
    virtual Scalar computeCalibration_rot(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

//    /** Compute Calibration (only translation).
//        \param sensor_poses initial calibration
//        \return the residual */
//    virtual Scalar computeCalibration_trans(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

};
