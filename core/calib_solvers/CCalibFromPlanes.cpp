
/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "CCalibFromPlanes.h"
#include <mrpt/poses/CPose3D.h>

#include <mrpt/pbmap/PbMap.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

CCalibFromPlanes::CCalibFromPlanes(CObservationTree *model, TCalibFromPlanesParams *params) :
    CExtrinsicCalib(model)
{
	for(int sensor_id1=0; sensor_id1 < sync_model->getNumberOfSensors(); sensor_id1++)
	{
		mvv_planes[sensor_id1] = std::vector<std::vector<CPlaneCHull>>();
		mmv_plane_corresp[sensor_id1] = std::map<int, std::vector<std::array<int,3>>>();
		for(int sensor_id2 = sensor_id1 + 1; sensor_id2 < sync_model->getNumberOfSensors(); sensor_id2++)
			mmv_plane_corresp[sensor_id1][sensor_id2] = std::vector<std::array<int,3>>();
	}

	this->params = params;
}

void CCalibFromPlanes::segmentPlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::vector<CPlaneCHull> & planes)
{
	unsigned min_inliers = params->seg.min_inliers_frac * cloud->size();

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;

	if(params->seg.normal_estimation_method == 0)
		normal_estimation.setNormalEstimationMethod(normal_estimation.COVARIANCE_MATRIX);
	else if(params->seg.normal_estimation_method == 1)
		normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_3D_GRADIENT);
	else
		normal_estimation.setNormalEstimationMethod(normal_estimation.AVERAGE_DEPTH_CHANGE);

	normal_estimation.setDepthDependentSmoothing(params->seg.depth_dependent_smoothing);
	normal_estimation.setMaxDepthChangeFactor(params->seg.max_depth_change_factor);
	normal_estimation.setNormalSmoothingSize(params->seg.normal_smoothing_size);

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setInputCloud(cloud);
	normal_estimation.compute(*normal_cloud);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> multi_plane_segmentation;
	multi_plane_segmentation.setMinInliers(min_inliers);
	multi_plane_segmentation.setAngularThreshold(params->seg.angle_threshold);
	multi_plane_segmentation.setDistanceThreshold(params->seg.dist_threshold);
	multi_plane_segmentation.setInputNormals(normal_cloud);
	multi_plane_segmentation.setInputCloud(cloud);

	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> regions;
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> unique_regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;

	multi_plane_segmentation.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

	// Create a vector with the planes detected in this frame, and calculate their parameters (normal, center, pointclouds, etc.)

	mrpt::pbmap::PbMap pbmap;

	for (size_t i = 0; i < regions.size(); i++)
	{
		CPlaneCHull planehull;

		if(regions[i].getCurvature() > params->seg.max_curvature)
			continue;

		mrpt::pbmap::Plane plane;
		plane.v3center = regions[i].getCentroid ();
		plane.v3normal = Eigen::Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
		plane.d = model_coefficients[i].values[3];

		// Force the normal vector to point towards the camera
		if( model_coefficients[i].values[3] < 0)
		{
			plane.v3normal = -plane.v3normal;
			plane.d = -plane.d;
		}

		plane.curvature = regions[i].getCurvature();

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud ( cloud );
		extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
		extract.setNegative (false);
		extract.filter (*plane.planePointCloudPtr);
		plane.inliers = inlier_indices[i].indices;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		contourPtr->points = regions[i].getContour();
		plane.calcConvexHull(contourPtr);

		// Check whether this region correspond to the same plane as a previous one (this situation may happen when there exists a small discontinuity in the observation)

		bool isSamePlane = false;
		for (size_t j = 0; j < pbmap.vPlanes.size(); j++)
			if(pbmap.vPlanes[j].isSamePlane(plane, params->seg.max_cos_normal, params->seg.dist_centre_plane_threshold, params->seg.proximity_threshold))
			{
				isSamePlane = true;
				pbmap.vPlanes[j].mergePlane(plane);
				boundary_indices.erase(boundary_indices.begin() + j);
				break;
			}

		if(!isSamePlane)
		{
			pbmap.vPlanes.push_back(plane);
			unique_regions.push_back(regions[i]);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	planes.resize(pbmap.vPlanes.size());
	for (size_t i = 0; i < pbmap.vPlanes.size (); i++)
	{
		planes[i].v3normal = pbmap.vPlanes[i].v3normal;
		planes[i].v3center = pbmap.vPlanes[i].v3center;
		planes[i].d = pbmap.vPlanes[i].d;

		planes[i].ConvexHullPtr = pbmap.vPlanes[i].polygonContourPtr;
		planes[i].v_inliers = pbmap.vPlanes[i].inliers;


		contourPtr->points = unique_regions[i].getContour();
		pbmap.vPlanes[i].calcConvexHull(contourPtr, planes[i].v_hull_indices);
		for (size_t j = 0; j < planes[i].v_hull_indices.size(); j++)
			planes[i].v_hull_indices[j] = boundary_indices[i].indices[planes[i].v_hull_indices[j]];
	}
}

void CCalibFromPlanes::findPotentialMatches(const std::vector<std::vector<CPlaneCHull>> &planes, const int &set_id)
{
	std::vector<Eigen::Vector2f> sensor_pose_uncertainties = sync_model->getSensorUncertainties();

	for(int i = 0; i < planes.size()-1; ++i)
		for(int j = i+1; j < planes.size(); ++j)
		{
			for(int ii = 0; ii < planes[i].size(); ++ii)
			{
				for(int jj = 0; jj < planes[j].size(); ++jj)
				{
					Eigen::Vector3f n_ii = sync_model->getSensorPoses()[i].block(0,0,3,3) * planes[i][ii].v3normal;
                    Scalar d1 = planes[i][ii].d - Eigen::Vector3f(sync_model->getSensorPoses()[i].block(0,3,3,1)).dot(n_ii);
					Eigen::Vector3f n_jj = sync_model->getSensorPoses()[j].block(0,0,3,3) * planes[j][jj].v3normal;
                    Scalar d2 = planes[j][jj].d - Eigen::Vector3f(sync_model->getSensorPoses()[j].block(0,3,3,1)).dot(n_jj);
					if((n_ii.dot(n_jj) > cos(sensor_pose_uncertainties[j][0] * (M_PI/180))) && (d1 - d2 < sensor_pose_uncertainties[j][1]))
					{
						std::array<int,3> potential_match{set_id, ii, jj};
						mmv_plane_corresp[i][j].push_back(potential_match);
					}
				}
			}

//			// for stats printing when no matches exist
//			if((planes[i].size() == 0) || (planes[j].size() == 0))
//			{
//				std::array<int,3> temp_match{-1, -1, -1};
//				mmv_plane_corresp[i][j].push_back(temp_match);
//			}
		}
}

Scalar CCalibFromPlanes::computeRotationResidual()
{
	std::vector<Eigen::Matrix4f> sensor_poses = sync_model->getSensorPoses();
	Scalar sum_squared_error = 0.; // Accumulated squared error for all plane correspondences

	for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator it_sensor_i = mmv_plane_corresp.begin();
	    it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
	{
		size_t sensor_i = it_sensor_i->first;
		for(std::map<int,std::vector<std::array<int,3>>>::iterator it_sensor_j = it_sensor_i->second.begin();
		    it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
		{
			size_t sensor_j = it_sensor_j->first;
			std::vector<std::array<int,3>> &correspondences = it_sensor_j->second;
			for(int i = 0; i < correspondences.size(); i++)
			{
				int set_id = correspondences[i][0];
				int sync_obs1_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_i]);
				int sync_obs2_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_j]);

				Eigen::Vector3f n_obs_i = mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]].v3normal;
				Eigen::Vector3f n_obs_j = mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]].v3normal;

				Eigen::Vector3f n_i = sensor_poses[sensor_i].block(0,0,3,3) * n_obs_i;
				Eigen::Vector3f n_j = sensor_poses[sensor_j].block(0,0,3,3) * n_obs_j;

				Eigen::Vector3f rot_error = (n_i - n_j);
				sum_squared_error += rot_error.dot(rot_error);
			}
		}
	}

	return sum_squared_error;
}

Scalar CCalibFromPlanes::computeRotation()
{
	std::vector<Eigen::Matrix4f> sensor_poses = sync_model->getSensorPoses();
	const int num_sensors = sensor_poses.size();
	const int dof = 3 * (num_sensors - 1);
	Eigen::VectorXf update_vector(dof);
	Eigen::Matrix3f jacobian_rot_i, jacobian_rot_j; // Jacobians of the rotation
	float error, new_error, init_error;
	float av_angle_error;

	std::vector<Eigen::Matrix4f> estimated_poses = sensor_poses;
	std::vector<Eigen::Matrix4f> estimated_poses_temp(sensor_poses.size());

	float increment = 1000, diff_error = 1000;
	int it = 0;

	init_error = computeRotationResidual();

	while(it < params->solver.max_iters && increment > params->solver.min_update && diff_error > params->solver.converge_error)
	{
		// Calculate the hessian and the gradient
		hessian = Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>::Zero(dof, dof); // Hessian of the rotation of the decoupled system
		gradient = Eigen::Matrix<Scalar,Eigen::Dynamic,1>::Zero(dof); // Gradient of the rotation of the decoupled system
		error = 0.0;
		av_angle_error = 0.0;

		for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator it_sensor_i = mmv_plane_corresp.begin();
		    it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
		{
			size_t sensor_i = it_sensor_i->first;
			for(std::map<int,std::vector<std::array<int,3>>>::iterator it_sensor_j = it_sensor_i->second.begin();
			    it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
			{
				size_t sensor_j = it_sensor_j->first;
				int pos_sensor_i = 3 * (sensor_i - 1);
				int pos_sensor_j = 3 * (sensor_j - 1);

				std::vector<std::array<int,3>> &correspondences = it_sensor_j->second;
				for(int i = 0; i < correspondences.size(); i++)
				{
					size_t set_id = correspondences[i][0];

					int sync_obs1_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_i]);
					int sync_obs2_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_j]);

					Eigen::Vector3f n_obs_i = mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]].v3normal;
					Eigen::Vector3f n_obs_j = mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]].v3normal;

					Eigen::Vector3f n_i = sensor_poses[sensor_i].block(0,0,3,3) * n_obs_i;
					Eigen::Vector3f n_j = sensor_poses[sensor_j].block(0,0,3,3) * n_obs_j;

					jacobian_rot_i = -skew(n_i);
					jacobian_rot_j = skew(n_j);

					Eigen::Vector3f rot_error = (n_i - n_j);
					error += rot_error.dot(rot_error);
					av_angle_error += acos(n_i.dot(n_j));

					if(sensor_i != 0) // The pose of the first camera is fixed
					{
						hessian.block(pos_sensor_i, pos_sensor_i, 3, 3) += jacobian_rot_i.transpose() * jacobian_rot_i;
						gradient.block(pos_sensor_i,0,3,1) += jacobian_rot_i.transpose() * rot_error;

						// Cross term
						hessian.block(pos_sensor_i, pos_sensor_j, 3, 3) += jacobian_rot_i.transpose() * jacobian_rot_j;
					}

					hessian.block(pos_sensor_j, pos_sensor_j, 3, 3) += jacobian_rot_j.transpose() * jacobian_rot_j;
					gradient.block(pos_sensor_j,0,3,1) += jacobian_rot_j.transpose() * rot_error;
				}

				if(sensor_i != 0) // Fill the lower left triangle with the corresponding cross terms
					hessian.block(pos_sensor_j, pos_sensor_i, 3, 3) = hessian.block(pos_sensor_i, pos_sensor_j, 3, 3).transpose();
			}
		}

		//cout << "hessian \n" << hessian << endl;
		//cout << "gradient \n" << gradient.transpose() << endl;
		//cout << "Error accumulated " << accum_error2 << endl;
		
		Eigen::FullPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> lu(hessian);
		if(!lu.isInvertible())
		{
			result.msg = "System is badly conditioned. Please try again with a new set of observations.";
			break;
		}

		// Update rotation
		update_vector = -hessian.inverse() * gradient;
		//cout << "update_vector " << update_vector.transpose() << endl;

		for(int sensor_id = 1; sensor_id < num_sensors; sensor_id++)
		{
			mrpt::poses::CPose3D pose;
			mrpt::math::CArrayNumeric<double,3> rot_manifold;
			rot_manifold[0] = update_vector(3*sensor_id-3,0);
			rot_manifold[1] = update_vector(3*sensor_id-2,0);
			rot_manifold[2] = update_vector(3*sensor_id-1,0);
			mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
			//cout << "update_rot\n" << update_rot << endl;
			Eigen::Matrix3f update_rot_eig;
			update_rot_eig << update_rot(0,0), update_rot(0,1), update_rot(0,2),
			        update_rot(1,0), update_rot(1,1), update_rot(1,2),
			        update_rot(2,0), update_rot(2,1), update_rot(2,2);
			estimated_poses_temp[sensor_id] = estimated_poses[sensor_id];
			estimated_poses_temp[sensor_id].block(0,0,3,3) = update_rot_eig * estimated_poses[sensor_id].block(0,0,3,3);
			//cout << "old rotation" << sensor_id << "\n" << Rt_estimated[sensor_id].block(0,0,3,3) << endl;
			//cout << "new rotation\n" << Rt_estimatedTemp[sensor_id].block(0,0,3,3) << endl;
		}

        //accum_error = computeRotationResidual(estimated_poses);
		//cout << "Error accumulated " << accum_error2 << endl;
		new_error = computeRotationResidual();

		//cout << "New rotation error " << new_accum_error2 << endl;
		//cout << "Closing loop? \n" << Rt_estimated[0].inverse() * Rt_estimated[7] * Rt_78;

		//Assign new rotations
		if(new_error < error)
			for(int sensor_id = 1; sensor_id < num_sensors; sensor_id++)
				estimated_poses[sensor_id] = estimated_poses_temp[sensor_id];

		increment = update_vector.dot(update_vector);
		diff_error = error - new_error;
		++it;
		//cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
	}

	if(it == params->solver.max_iters)
		result.msg = "Maximum iterations reached.";
	else if(increment < params->solver.min_update)
		result.msg = "Increment too small.";
	else if(diff_error < params->solver.converge_error)
		result.msg = "Convergence";

	result.init_error = init_error;
	result.num_iters = it;
	result.final_error = new_error;
	result.estimate = estimated_poses;

	//std::cout << "ErrorCalibRotation " << accum_error2/numPlaneCorresp << " " << av_angle_error/numPlaneCorresp << std::endl;
}

Scalar CCalibFromPlanes::computeTranslation()
{
	std::vector<Eigen::Matrix4f> sensor_poses = sync_model->getSensorPoses();
    const int num_sensors = sensor_poses.size();
    const int dof = 3 * (num_sensors - 1);
    Eigen::VectorXf update_vector(dof);
    Eigen::Matrix3f jacobian_trans_i, jacobian_trans_j; // Jacobians of the rotation
    float accum_error2(0.f);

    for(std::map<int,std::map<int,std::vector<std::array<int,3>>>>::iterator it_sensor_i = mmv_plane_corresp.begin();
        it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
    {
        size_t sensor_i = it_sensor_i->first;
        for(std::map<int,std::vector<std::array<int,3>>>::iterator it_sensor_j = it_sensor_i->second.begin();
            it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
        {
            size_t sensor_j = it_sensor_j->first;
            int pos_sensor_i = 3 * (sensor_i - 1);
            int pos_sensor_j = 3 * (sensor_j - 1);

            std::vector<std::array<int,3>> &correspondences = it_sensor_j->second;
            for(int i = 0; i < correspondences.size(); i++)
            {
                size_t set_id = correspondences[i][0];

                int sync_obs1_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_i]);
                int sync_obs2_id = sync_model->findSyncIndexFromSet(set_id, sync_model->getSensorLabels()[sensor_j]);

                Eigen::Vector3f n_obs_i = mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]].v3normal;
                Eigen::Vector3f n_obs_j = mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]].v3normal;

                Eigen::Vector3f n_i = sensor_poses[sensor_i].block(0,0,3,3) * n_obs_i;
                Eigen::Vector3f n_j = sensor_poses[sensor_j].block(0,0,3,3) * n_obs_j;

                float trans_error = mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]].d - mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]].d; // Consider all zero initial translations
                //trans_error = (mvv_planes[sensor_i][sync_obs1_id][correspondences[i][1]].d - Eigen::Vector3fsensor_poses[sensor_i].block(0,3,3,1)).dot(n_i) - (mvv_planes[sensor_j][sync_obs2_id][correspondences[i][2]].d - Eigen::Vector3f(sensor_poses[sensor_j].block(0,3,3,1)).dot(n_j));
                accum_error2 += trans_error * trans_error;

                jacobian_trans_i = n_i * n_i.transpose();
                jacobian_trans_j = -n_i * n_j.transpose();

                if(sensor_i != 0) // The pose of the first camera is fixed
                {
                    hessian.block(pos_sensor_i, pos_sensor_i, 3, 3) += jacobian_trans_i.transpose() * jacobian_trans_i;
                    gradient.block(pos_sensor_i,0,3,1) += jacobian_trans_i.transpose() * trans_error;

                    // Cross term
                    hessian.block(pos_sensor_i, pos_sensor_j, 3, 3) += jacobian_trans_i.transpose() * jacobian_trans_j;
                }

                hessian.block(pos_sensor_j, pos_sensor_j, 3, 3) += jacobian_trans_j.transpose() * jacobian_trans_j;
                gradient.block(pos_sensor_j,0,3,1) += jacobian_trans_j.transpose() * trans_error;
            }

            if(sensor_i != 0) // Fill the lower left triangle with the corresponding cross terms
                hessian.block(pos_sensor_j, pos_sensor_i, 3, 3) = hessian.block(pos_sensor_i, pos_sensor_j, 3, 3).transpose();
        }
    }

    //cout << "hessian \n" << hessian << endl;
    //cout << "gradient \n" << gradient.transpose() << endl;
    //cout << "Error accumulated " << accum_error2 << endl;

    Eigen::FullPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> lu(hessian);
    if(!lu.isInvertible())
    {
		result.msg = "System is badly conditioned. Please try again with a new set of observations.";
        return accum_error2;
    }

    // Update the translation (Linear Least-Squares -> exact solution)
    update_vector = -hessian.inverse() * gradient;
    //cout << "update_vector " << update_vector.transpose() << endl;

//    for(int sensor_id = 1; sensor_id < num_sensors; sensor_id++)
//        sensor_poses[sensor_id].block(0,3,3,1) = update_vector.block(3*(sensor_id-1),0,3,1);

    return accum_error2;
}
