/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include "TExtrinsicCalibParams.h"
#include <CObservationTree.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

typedef float Scalar;

/*! Generate a skew-symmetric matrix from a 3D vector */
template<typename Scalar> inline Eigen::Matrix<Scalar,3,3> skew(const Eigen::Matrix<Scalar,3,1> &vec)
{
  Eigen::Matrix<Scalar,3,3> skew_matrix = Eigen::Matrix<Scalar,3,3>::Zero();
  skew_matrix(0,1) = -vec(2);
  skew_matrix(1,0) = vec(2);
  skew_matrix(0,2) = vec(1);
  skew_matrix(2,0) = -vec(1);
  skew_matrix(1,2) = -vec(0);
  skew_matrix(2,1) = vec(0);
  return skew_matrix;
}

/** Base class for extrinsic calibration.
 *
 * \tparam num_sensors The number of sensors to calibrate (i.e. number of sensors
 * in the system - 1)
 * \tparam Scalar The precision of matrix elements, thresholds, etc. (typ: float or
 * double). Defaults to double.
 */

class CExtrinsicCalib
{
public:
    /** Default constructor. */
	CExtrinsicCalib(CObservationTree *model)
	{
		sync_model = model;
	}

    /** Destructor: */
	virtual ~CExtrinsicCalib(){}

	CObservationTree *sync_model;

    /** Load the initial calibration (into the static members).
        \param init_calib_file containing the initial calibration */
    void loadInitialCalibration(const std::string init_calib_file);

    /** Save calibration.
        \param file_path to save the calibration */
    void saveCalibration(const std::string file_path);

    /** Calculate the residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
//    virtual Scalar computeCalibResidual(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses) = 0;

    /** Calculate the angular residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
	virtual Scalar computeRotationResidual() = 0;

//    /** Calculate the translational residual error of the correspondences.
//        \param sensor_poses relative poses of the sensors
//        \return the residual */
//    virtual Scalar computeTranslationResidual(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses) = 0;

    /** Compute Calibration.
        \param sensor_poses initial calibration
        \return the residual */
	virtual Scalar computeCalibration(); // = 0;

    /** Compute Calibration (only rotation).
	 * \params params the parameters related to the least-squares solver
	 * \return the residual */
	virtual Scalar computeRotation() = 0;

    /** Compute Calibration (only translation).
        \return the residual */
	virtual Scalar computeTranslation() = 0;

//    /*! Compute the Fisher Information Matrix (FIM) of the rotation estimate. */ // TODO
//    void calcFIM_rot();

//private:
    /** Threshold to discard the calibration when the FIM is ill conditioned: smallest_eig/biggest_eig < threshold. */
    static double eigenvalue_ratio_threshold;

    /*! Conditioning numbers that indicate how reliable is the information to calculate the extrinsic calibration */
    //std::array<Scalar, num_sensors> m_conditioning;
    std::vector<Scalar> m_conditioning;

    /** The pose of the reference sensor wrt. the global reference system (i.e. the vehicle / robot platform). */
    static mrpt::math::CMatrixFixedNumeric<Scalar,4,4> ref_sensor_pose;

    /** The estimated calibration (relative poses of the sensor systems wrt. the reference one) */
	std::vector<Eigen::Matrix4f> m_calibration;
    //std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > m_calibration(num_methods, Eigen::Affine3f::Identity());

    /** The estimated calibration's uncertainty */
    std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,6,6> > m_calib_uncertainty;

    /** Hessian of the of the least-squares problem */
    Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> hessian;

    /** Gradient of the of the least-squares problem */
    Eigen::Matrix<Scalar,Eigen::Dynamic,1> gradient;
};
