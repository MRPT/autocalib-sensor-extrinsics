#pragma once

#include <algorithm>
#include <vector>
#include <mrpt/math/CMatrix.h>
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

	/** Function template to get so(3) rotation from SE(3) transformation. */
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

	/** Function template to get translation from SE(3) transformation. */
	template <typename T>
	Eigen::Matrix<T,3,1> getTranslationVector(const Eigen::Matrix<T,4,4> &sensor_pose)
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
	void backprojectTo3D(T &point,  mrpt::math::CMatrix &range, const mrpt::img::TCamera &params, S &point3D)
	{
		point3D[2] = range(point[1], point[0]);
		point3D[0] = ((point[0] - params.cx())/params.fx()) * point3D[2];
		point3D[1] = ((point[1] - params.cy())/params.fy()) * point3D[2];
	}
}
