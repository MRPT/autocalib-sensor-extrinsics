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
#include "TCalibFromPlanesParams.h"
#include <CPlanes.h>
//#include <mrpt/pbmap/PbMap.h>
//#include <mrpt/pbmap/Miscellaneous.h>
#include <map>

/** Exploit 3D plane observations from a set of sensors to perform extrinsic calibration.
 *  Plane correspondences are analogous to the control points used to create panoramic images with a regular camera).
 *  It allows to estimate the extrinsic calibration between RGB-D sensors like Asus XPL.
 */

class CCalibFromPlanes : public CExtrinsicCalib
{
  public:

	/** The segmented planes, the vector indices to access them are [sensor_id][obs_id][plane_id]
	 * obs_id is with respect to the synchronized model.
	 */
	std::map<int,std::vector<std::vector<CPlaneCHull>>> mvv_planes;

	/** The plane correspondences between the different sensors.
	 * The map indices correspond to the sensor ids, with the list of correspondeces
	 * stored as a matrix with each row of the form - set_id, plane_id1, plane_id2.
	 */
	std::map<int,std::map<int,std::vector<std::array<int,3>>>> mmv_plane_corresp;

	/*! Covariance matrices */
    std::vector< Eigen::Matrix<Scalar,3,3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar,3,3> > > covariance_rot;
    std::vector< Eigen::Matrix<Scalar,3,3>, Eigen::aligned_allocator<Eigen::Matrix<Scalar,3,3> > > m_covariance_trans;

	/*! Constructor */
	CCalibFromPlanes(CObservationTree *model);

    /*! Destructor */
	virtual ~CCalibFromPlanes(){}

	/**
	 * \brief Runs pcl's organized multi-plane segmentation over the given cloud.
	 * @param cloud the input cloud.
	 * @param params the parameters for segmentation.
	 * @param planes the segmented planes.
	 */
	void segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const TPlaneSegmentationParams &params, std::vector<CPlaneCHull> &planes);

	/**
	 * Search for potential plane matches.
	 * \param planes planes extracted from sensor observations that belong to the same synchronized set. planes[sensor_id][plane_id] gives a plane.
	 * \param set_id the id of the synchronized set the planes belong to.
	 * \param params the parameters for plane matching.
	 */
	void findPotentialMatches(const std::vector<std::vector<CPlaneCHull>> &planes, const int &set_id, const TPlaneMatchingParams &params);

    /** Calculate the residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    //virtual Scalar computeCalibResidual(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);
//    virtual Scalar computeCalibResidual(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

    /** Calculate the angular residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
	virtual Scalar computeRotCalibResidual(const std::vector<Eigen::Matrix4f> &sensor_poses);

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
	virtual Scalar computeRotCalibration(const TSolverParams &params, const std::vector<Eigen::Matrix4f> &sensor_poses, std::string &stats);

//    /** Compute Calibration (only translation).
//        \param sensor_poses initial calibration
//        \return the residual */
//    virtual Scalar computeCalibration_trans(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses);

};
