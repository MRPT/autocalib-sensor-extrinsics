/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include "extrinsic_calib.h"
//#include <mrpt/pbmap/PbMap.h>
//#include <mrpt/pbmap/Miscellaneous.h>

/*! Exploit 3D plane observations from set a of sensors to perform extrinsic calibration.
 *  Plane correspondences are analogous to the control points used to create panoramic images with a regular camera).
 *  It allows to estimate the extrinsic calibration between RGB-D sensors like Asus XPL.
 *
 */
template <int num_sensors, typename Scalar = double>
class CalibFromPlanes3D : public ExtrinsicCalib<num_sensors, Scalar>
{
  public:

    /*! The plane correspondences between the different sensors. The map indexes correspond to the sensor IDs,
     * each matrix as as many rows as potential plane correspondences, with 4 columns for: obs_id1, plane_id1, obs_id2, plane_id2. */
    std::map<unsigned, std::map<unsigned, mrpt::math::CMatrix<unsigned> > > mm_plane_corresp;

    /*! Rotation covariance matrices */
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > m_covariances;

    /*! Constructor */
    CalibFromPlanes3D(){}

    /*! Destructor */
    virtual ~CalibFromPlanes3D(){}

    /** Calculate the residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual(std::array<mrpt::math::CMatrixFixedNumeric<typename Scalar,4,4>, typename num_sensors> sensor_poses);

    /** Calculate the angular residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual_rot()(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);

    /** Calculate the translational residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual_trans()(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);

    /** Compute Calibration.
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);

    /** Compute Calibration (only rotation).
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration_rot(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);

    /** Compute Calibration (only translation).
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration_trans(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses);

};
