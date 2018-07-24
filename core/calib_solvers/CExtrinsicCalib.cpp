/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "CExtrinsicCalib.h"

//template <int num_sensors, typename Scalar>
//Scalar CExtrinsicCalib<num_sensors,Scalar>::eigenvalue_ratio_threshold = 2e-4;
double CExtrinsicCalib::eigenvalue_ratio_threshold = 2e-4;

//template <int num_sensors, typename Scalar>
//CExtrinsicCalib<num_sensors,Scalar>::CExtrinsicCalib()
//CExtrinsicCalib::CExtrinsicCalib(size_t n_sensors)
//{
//    //m_conditioning.fill(0.);
//    //    covariances_ = std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >(n_sensors, Eigen::Matrix3f::Zero());

//}
