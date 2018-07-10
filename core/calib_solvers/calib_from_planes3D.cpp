
/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "calib_from_planes3D.h"
#include <mrpt/poses/CPose3D.h>

using namespace std;

//template <int num_sensors, typename Scalar>
//CalibFromPlanes3D<num_sensors,Scalar>::CalibFromPlanes3D()
CalibFromPlanes3D::CalibFromPlanes3D(size_t n_sensors) : ExtrinsicCalib(n_sensors)
{
    std::cout << "CalibFromPlanes3D... num_sensors " << num_sensors << std::endl;
    for(size_t sensor_id1=0; sensor_id1 < num_sensors; sensor_id1++)
    {
        mmv_plane_corresp[sensor_id1] = std::map<size_t, std::vector< std::array<size_t,4> > >();
        for(size_t sensor_id2=sensor_id1+1; sensor_id2 < num_sensors; sensor_id2++)
            mmv_plane_corresp[sensor_id1][sensor_id2] = std::vector< std::array<size_t,4> >();
    }
}

void CalibFromPlanes3D::findPotentialMatches(const std::vector< std::vector< PlaneCHull > > & plane_obs, size_t obs_id)
{
    for (size_t i = 0; i < plane_obs.size()-1; ++i)     // Sensor i
        for (size_t j = i+1; j < plane_obs.size(); ++j) // Sensor j
            for (size_t ii = 0; ii < plane_obs[i].size(); ++ii)
                for (size_t jj = 0; jj < plane_obs[j].size(); ++jj)
                {
                    Eigen::Vector3f n_ii = ExtrinsicCalib::m_init_calib[i].block(0,0,3,3)*plane_obs[i][ii].v3normal;
                    Eigen::Vector3f n_jj = ExtrinsicCalib::m_init_calib[j].block(0,0,3,3)*plane_obs[j][jj].v3normal;
                    if( n_ii.dot(n_jj) > 0.9 ) // TODO, define constraint threshold in a config file, and allow user interaction
                    {
                        std::array<size_t,4> potential_match{obs_id, obs_id, ii, jj};
                        mmv_plane_corresp[i][j].push_back(potential_match);
                    }
                }
}

Scalar CalibFromPlanes3D::computeCalibResidual_rot(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses)
{
    Scalar sum_squared_error = 0.; // Accumulated squared error for all plane correspondences
    for(std::map<size_t, std::map<size_t, std::vector< std::array<size_t,4> > > >::iterator it_sensor_i = mmv_plane_corresp.begin();
        it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
    {
        size_t sensor_i = it_sensor_i->first;
        for(std::map<size_t, std::vector< std::array<size_t,4> > >::iterator it_sensor_j = it_sensor_i->second.begin();
            it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
        {
            size_t sensor_j = it_sensor_j->first;
            std::vector< std::array<size_t,4> > & correspondences = it_sensor_j->second;
            for(size_t i=0; i < correspondences.size(); i++) // For every potential match
            {
                size_t frame_id = correspondences[i][0];
                Eigen::Vector3f n_obs_i = vvv_planes[frame_id][sensor_i][correspondences[i][2]].v3normal;
                Eigen::Vector3f n_obs_j = vvv_planes[frame_id][sensor_j][correspondences[i][3]].v3normal;
                Eigen::Vector3f n_i = sensor_poses[sensor_i].block(0,0,3,3) * n_obs_i;
                Eigen::Vector3f n_j = sensor_poses[sensor_j].block(0,0,3,3) * n_obs_j;
                Eigen::Vector3f rot_error = (n_i - n_j);
                sum_squared_error += rot_error.dot(rot_error);
            }
        }
    }
    return sum_squared_error;
}

Scalar CalibFromPlanes3D::computeCalibration_rot(const std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > & sensor_poses)
{
    cout << "CalibFromPlanes3D::computeCalibration_rot...\n";
    const int n_DoF = 3 * (num_sensors - 1);
    Eigen::VectorXf update_vector(n_DoF);
    Eigen::Matrix3f jacobian_rot_i, jacobian_rot_j; // Jacobians of the rotation
    float accum_error2;
    float av_angle_error;
    unsigned numPlaneCorresp;

    std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > Rt_estimated = sensor_poses;
    std::vector<mrpt::math::CMatrixFixedNumeric<Scalar,4,4> > Rt_estimatedTemp;

    // Parameters of the Least-Squares optimization
    int _max_iterations = 10;
    float _epsilon_transf = 0.00001;
    float _convergence_error = 0.000001;
    float increment = 1000, diff_error = 1000;
    int it = 0;
    while(it < _max_iterations && increment > _epsilon_transf && diff_error > _convergence_error)
    {
        // Calculate the hessian and the gradient
        hessian = Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>::Zero(n_DoF,n_DoF); // Hessian of the rotation of the decoupled system
        gradient = Eigen::Matrix<Scalar,Eigen::Dynamic,1>::Zero(n_DoF); // Gradient of the rotation of the decoupled system
        accum_error2 = 0.0;
        av_angle_error = 0.0;
        numPlaneCorresp = 0;

        for(std::map<size_t, std::map<size_t, std::vector< std::array<size_t,4> > > >::iterator it_sensor_i = mmv_plane_corresp.begin();
            it_sensor_i != mmv_plane_corresp.end(); it_sensor_i++)
        {
            size_t sensor_i = it_sensor_i->first;
            for(std::map<size_t, std::vector< std::array<size_t,4> > >::iterator it_sensor_j = it_sensor_i->second.begin();
                it_sensor_j != it_sensor_i->second.end(); it_sensor_j++)
            {
                size_t sensor_j = it_sensor_j->first;
                int pos_sensor_i = 3*(sensor_i - 1);
                int pos_sensor_j = 3*(sensor_j - 1);
                std::vector< std::array<size_t,4> > & correspondences = it_sensor_j->second;
                for(size_t i=0; i < correspondences.size(); i++) // For every potential match
                {
                    size_t frame_id = correspondences[i][0];
                    Eigen::Vector3f n_obs_i = vvv_planes[frame_id][sensor_i][correspondences[i][2]].v3normal;
                    Eigen::Vector3f n_obs_j = vvv_planes[frame_id][sensor_j][correspondences[i][3]].v3normal;
                    Eigen::Vector3f n_i = sensor_poses[sensor_i].block(0,0,3,3) * n_obs_i;
                    Eigen::Vector3f n_j = sensor_poses[sensor_j].block(0,0,3,3) * n_obs_j;
                    jacobian_rot_i = -skew(n_i);
                    jacobian_rot_j = skew(n_j);
                    Eigen::Vector3f rot_error = (n_i - n_j);
                    accum_error2 += rot_error.dot(rot_error);
                    av_angle_error += acos(n_i.dot(n_j));
                    numPlaneCorresp++;

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
        cout << "Error accumulated " << accum_error2 << endl;

//        if( calcConditioning() > threshold_conditioning_ )
//        {
//            cout << "\tRotation system is bad conditioned " << conditioning_ << " threshold " << threshold_conditioning_ << "\n";
//            break;
//        }

        // Update rotation
        update_vector = -hessian.inverse() * gradient;
        cout << "update_vector " << update_vector.transpose() << endl;
        for(int sensor_id = 1; sensor_id < num_sensors; sensor_id++)
        {
            mrpt::poses::CPose3D pose;
            mrpt::math::CArrayNumeric< double, 3 > rot_manifold;
            rot_manifold[0] = update_vector(3*sensor_id-3,0);
            rot_manifold[1] = update_vector(3*sensor_id-2,0);
            rot_manifold[2] = update_vector(3*sensor_id-1,0);
            mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
            //cout << "update_rot\n" << update_rot << endl;
            Eigen::Matrix3f update_rot_eig;
            update_rot_eig << update_rot(0,0), update_rot(0,1), update_rot(0,2),
                                update_rot(1,0), update_rot(1,1), update_rot(1,2),
                                update_rot(2,0), update_rot(2,1), update_rot(2,2);
            Rt_estimatedTemp[sensor_id] = Rt_estimated[sensor_id];
            Rt_estimatedTemp[sensor_id].block(0,0,3,3) = update_rot_eig * Rt_estimated[sensor_id].block(0,0,3,3);
            //      cout << "old rotation" << sensor_id << "\n" << Rt_estimated[sensor_id].block(0,0,3,3) << endl;
            //      cout << "new rotation\n" << Rt_estimatedTemp[sensor_id].block(0,0,3,3) << endl;
        }

        accum_error2 = computeCalibResidual_rot(Rt_estimated);
        cout << "Error accumulated " << accum_error2 << endl;
        float new_accum_error2 = computeCalibResidual_rot(Rt_estimatedTemp);
        //        float new_accum_error2 = calcCorrespRotError(Rt_estimatedTemp);

        //        cout << "New rotation error " << new_accum_error2 << endl;
        //    cout << "Closing loop? \n" << Rt_estimated[0].inverse() * Rt_estimated[7] * Rt_78;

        // Assign new rotations
        if(new_accum_error2 < accum_error2)
            for(int sensor_id = 1; sensor_id < num_sensors; sensor_id++)
                Rt_estimated[sensor_id] = Rt_estimatedTemp[sensor_id];
        //            Rt_estimated[sensor_id].block(0,0,3,3) = Rt_estimatedTemp[sensor_id].block(0,0,3,3);

        increment = update_vector .dot (update_vector);
        diff_error = accum_error2 - new_accum_error2;
        ++it;
        cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
    }

    std::cout << "ErrorCalibRotation " << accum_error2/numPlaneCorresp << " " << av_angle_error/numPlaneCorresp << std::endl;
}
