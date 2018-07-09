/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "extrinsic_calib.h"

//template <int num_sensors, typename Scalar>
//Scalar ExtrinsicCalib<num_sensors,Scalar>::eigenvalue_ratio_threshold = 2e-4;
double ExtrinsicCalib::eigenvalue_ratio_threshold = 2e-4;

std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > ExtrinsicCalib::m_init_calib = std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> >(2);

//template <int num_sensors, typename Scalar>
//ExtrinsicCalib<num_sensors,Scalar>::ExtrinsicCalib()
//ExtrinsicCalib::ExtrinsicCalib(size_t n_sensors)
//{
//    //m_conditioning.fill(0.);
//    //    covariances_ = std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >(n_sensors, Eigen::Matrix3f::Zero());

//}
