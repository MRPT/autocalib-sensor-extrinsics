/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "extrinsic_calib.h"

ExtrinsicCalib::eigenvalue_ratio_threshold = 2e-4;

ExtrinsicCalib::ExtrinsicCalib()
{
    m_conditioning.fill(eigenvalue_ratio_threshold);
    //    covariances_ = std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> >(n_sensors, Eigen::Matrix3f::Zero());

}
