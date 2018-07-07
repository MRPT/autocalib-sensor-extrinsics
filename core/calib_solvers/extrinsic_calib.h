/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixFixedNumeric.h>

/** Base class for extrinsic calibration.
 *
 * \tparam num_sensors The number of sensors to calibrate (i.e. number of sensors
 * in the system - 1)
 * \tparam Scalar The precision of matrix elements, thresholds, etc. (typ: float or
 * double). Defaults to double.
 */
template <int num_sensors, typename Scalar = double>
class ExtrinsicCalib
{
public:
    /** Default constructor. */
    ExtrinsicCalib();

    /** Destructor: */
    virtual ~ExtrinsicCalib(){}

    /** Sensor labels. */
    std::array<std::string, num_sensors+1> sensor_labels;

    /** Load the initial calibration (into the static members).
        \param init_calib_file containing the initial calibration */
    void loadInitialCalibration(const std::string init_calib_file);

    /** Save calibration.
        \param file_path to save the calibration */
    void saveCalibration(const std::string file_path);

    /** Calculate the residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual(std::array<mrpt::math::CMatrixFixedNumeric<typename Scalar,4,4>, typename num_sensors> sensor_poses) = 0;

    /** Calculate the angular residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual_rot()(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses) = 0;

    /** Calculate the translational residual error of the correspondences.
        \param sensor_poses relative poses of the sensors
        \return the residual */
    Scalar computeCalibResidual_trans()(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses) = 0;

    /** Compute Calibration.
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses) = 0;

    /** Compute Calibration (only rotation).
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration_rot(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses) = 0;

    /** Compute Calibration (only translation).
        \param sensor_poses initial calibration
        \return the residual */
    Scalar computeCalibration_trans(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses) = 0;

//    /*! Compute the Fisher Information Matrix (FIM) of the rotation estimate. */ // TODO
//    void calcFIM_rot();

private:
    /** Threshold to discard the calibration when the FIM is ill conditioned: smallest_eig/biggest_eig < threshold. */
    static Scalar eigenvalue_ratio_threshold;

    /*! Conditioning numbers that indicate how reliable is the information to calculate the extrinsic calibration */
    std::array<Scalar, num_sensors> m_conditioning;

    /** The pose of the reference sensor wrt. the global reference system (i.e. the vehicle / robot platform). */
    static mrpt::math::CMatrixFixedNumeric<Scalar,4,4> ref_sensor_pose;

    /** The initial calibration (relative poses of the sensor systems wrt. the reference one) */
    static std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> m_init_calib;

    /** The estimated calibration (relative poses of the sensor systems wrt. the reference one) */
    std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> m_calibration;
    //std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f> > m_calibration(num_methods, Eigen::Affine3f::Identity());

    /** The estimated calibration's uncertainty */
    std::array<mrpt::math::CMatrixFixedNumeric<Scalar,6,6>, num_sensors> m_calib_uncertainty;
};
