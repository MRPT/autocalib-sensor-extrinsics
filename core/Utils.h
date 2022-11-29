#pragma once

#include <iostream>
#include <algorithm>
#include <vector>
#include <mrpt/img/TCamera.h>
#include <mrpt/poses/CPose3D.h>
#include <Eigen/Core>

namespace utils
{
    // Define some colours to draw bolobs, patches, etc.
    namespace colors
	{
	    static const unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
		static const unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
		static const unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};
	}

	/** Function template to find the position of an element in a vector. */
	template <typename T>
	size_t findItemIndexIn(const std::vector<T> &vec, const T &item)
	{
		auto iter = std::find(vec.begin(), vec.end(), item);
		if(iter != vec.end())
			return std::distance(vec.begin(), iter);
		else
			return -1;
	}

	/** Function template to get rotation angles from SE(3) transformation. */
	template <typename T>
	Eigen::Matrix<T,3,1> getRotations(const Eigen::Matrix<T,4,4> &sensor_pose)
	{
		Eigen::Matrix<T,3,1> rot_vec;
		mrpt::math::CMatrixDouble44 mrpt_mat(sensor_pose);
		mrpt::poses::CPose3D mrpt_pose(mrpt_mat);
		rot_vec[0] = (mrpt_pose.roll() * 180) / M_PI;
		rot_vec[1] = (mrpt_pose.pitch() * 180) / M_PI;
		rot_vec[2] = (mrpt_pose.yaw() * 180) / M_PI;
		return rot_vec;
	}

	/** Function template to convert rotation angles to SO(3). */
	template<typename T>
	Eigen::Matrix<T,3,3> getRotationMatrix(const Eigen::Matrix<T,3,1> &angles)
	{
		Eigen::Matrix<T,3,3> r, rx, ry, rz;

		rx << 1, 0, 0, 0, cos(angles(0)), -1 * sin(angles(0)), 0, sin(angles(0)), cos(angles(0));
		ry << cos(angles(1)), 0, sin(angles(1)), 0, 1, 0, -1 * sin(angles(1)), 0, cos(angles(1));
		rz << cos(angles(2)), -1 * sin(angles(2)), 0, sin(angles(2)), cos(angles(2)), 0, 0, 0, 1;

		r = rz * ry * rx;
		return r;
	}

	/** Function template to get translation from SE(3) transformation. */
	template <typename T>
	Eigen::Matrix<T,3,1> getTranslations(const Eigen::Matrix<T,4,4> &sensor_pose)
	{
		return sensor_pose.block(0,3,3,1);
	}

	template <typename T>
	void transformPoint(const Eigen::Affine3f &transform, T &point)
	{
		Eigen::Vector3f tpoint(3);
		tpoint(0) = point.x;
		tpoint(1) = point.y;
		tpoint(2) = point.z;

		tpoint = transform * tpoint;
		point.x = tpoint(0);
		point.y = tpoint(1);
		point.z = tpoint(2);
	}

	/**
	 * \brief Function template to back project 2D point to its corresponding 3D point in space
	 * \param point the 2D pixel coordinates
	 * \param range the depth image
	 * \param params the intrinsic parameters of the camera
	 * \param point3D the calculated 3D point in space
	 */
	template <typename T, typename S>
	void backprojectTo3D(T &point,  Eigen::MatrixXf &range, const mrpt::img::TCamera &params, S &point3D)
	{
		point3D[2] = range(point[1], point[0]);
		point3D[0] = ((point[0] - params.cx())/params.fx()) * point3D[2];
		point3D[1] = ((point[1] - params.cy())/params.fy()) * point3D[2];
	}
}
