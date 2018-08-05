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
#include <Eigen/Core>

typedef float Scalar;

/** Represents the extracted line along with its 2D and 3D geometrical characteristics. */

class CLine
{
public:

	Scalar rho;
	Scalar theta;

	Scalar m;
	Scalar c;

	std::array<Eigen::Vector2i,2> end_points;
	std::array<Eigen::Vector3f,2> end_points3D;
	Eigen::Vector2i mean_point;

	/** The 2D direction vector. [lx ly 0] */
	Eigen::Vector3i l;

	/** Equation of the 3D ray passing through the mean_point from the camera centre. */
	Eigen::Vector3f ray;
	/** Equation of the 3D normal of the projective plane. */
	Eigen::Vector3f normal;

	/** 3D coordinates of the mean_point. */
	Eigen::Vector3f p;
	/** 3D direction vector of the line. */
	Eigen::Vector3f v;
};
