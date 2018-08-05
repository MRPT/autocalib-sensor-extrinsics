/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <array>
#include <opencv2/core/core.hpp>

typedef float Scalar;

/** Represents the extracted line along with its 2D and 3D geometrical characteristics. */

class CLine
{
public:

	Scalar rho;
	Scalar theta;

	Scalar m;
	Scalar c;

	std::array<cv::Vec2i,2> end_points;
	std::array<cv::Vec3d,2> end_points3D;
	cv::Vec2i mean_point;

	/** The 2D direction vector. [lx ly 0] */
	cv::Vec3i l;

	/** Equation of the 3D ray passing through the mean_point from the camera centre. */
	cv::Vec3d ray;
	/** Equation of the 3D normal of the projective plane. */
	cv::Vec3d normal;

	/** 3D coordinates of the mean_point. */
	cv::Vec3d p;
	/** 3D direction vector of the line. */
	cv::Vec3d v;
};
