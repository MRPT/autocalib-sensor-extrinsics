#pragma once

#include <algorithm>
#include <vector>
#include <mrpt/math/CMatrix.h>
#include <mrpt/poses/CPose3D.h>
#include <Eigen/Core>

/** Function template to find the position of an element in a vector. */

// Define some colours to draw bolobs, patches, etc.
namespace cutils
{
    namespace colors
	{
	    static const unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
		static const unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
		static const unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};
	}

	template <typename T>
	size_t findItemIndexIn(const std::vector<T> &vec, const T &item)
	{
		auto iter = std::find(vec.begin(), vec.end(), item);
		return std::distance(vec.begin(), iter);
	}

	template <typename T>
	Eigen::Matrix<T,3,1> getRotationVector(const Eigen::Matrix<T,4,4> &sensor_pose)
	{
		Eigen::Matrix<T,3,1> rot_vec;
		mrpt::math::CMatrixDouble44 mrpt_mat(sensor_pose);
		mrpt::poses::CPose3D mrpt_pose(mrpt_mat);
		mrpt::math::CArrayDouble<3> rot_vec_mrpt = mrpt_pose.ln_rotation();
		rot_vec = Eigen::Matrix<T,3,1>(rot_vec_mrpt[0], rot_vec_mrpt[1], rot_vec_mrpt[2]);

		return rot_vec;
	}

	template <typename T>
	Eigen::Matrix<T,3,1> getTranslationVector(const Eigen::Matrix<T,4,4> &sensor_pose)
	{
		return sensor_pose.block(0,3,3,1);
	}
}
