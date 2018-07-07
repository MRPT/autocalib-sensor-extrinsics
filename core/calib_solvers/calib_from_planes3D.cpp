
/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "calib_from_planes3D.h"

using namespace std;

CalibFromPlanes3D::CalibFromPlanes3D()
{
    std::cout << "CalibFromPlanes3D... num_sensors " << sensor_labels.size()+1 << std::endl;
    unsigned n_sensors = sensor_labels.size()+1;
    for(unsigned sensor_id1=0; sensor_id1 < n_sensors; sensor_id1++)
    {
        mm_plane_corresp[sensor_id1] = std::map<unsigned, mrpt::math::CMatrix<unsigned> >();
        for(unsigned sensor_id2=sensor_id1+1; sensor_id2 < n_sensors; sensor_id2++)
            mm_plane_corresp[sensor_id1][sensor_id2] = mrpt::math::CMatrix<unsigned>(0, 4);
    }
}

Scalar CalibFromPlanes3D::computeCalibResidual_rot()(std::array<mrpt::math::CMatrixFixedNumeric<Scalar,4,4>, num_sensors> sensor_poses)
{
    Scalar sum_squared_error = 0.; // Accumulated squared error for all plane correspondences
    for(std::map<unsigned, std::map<unsigned, mrpt::math::CMatrix<unsigned> > >::iterator it_sensor1 = mm_plane_corresp.begin();
        it_sensor1 != mm_plane_corresp[sensor_id].end(); it_sensor1++)
    {
        unsigned sensor1 = it_sensor1->first;
        for(std::map<unsigned, mrpt::math::CMatrix<unsigned> >::iterator it_sensor2 = it_sensor1->second;
            it_sensor2 != it_sensor1->second.end(); it_sensor2++)
        {
            unsigned sensor2 = it_sensor2->first;
//            mrpt::math::CMatrix<unsigned> & corresp = it_sensor2->second;
//            for(unsigned i=0; i < it_pair->second.rows(); i++)
//            {
//                Eigen::Vector3f n_obs_1(...);
//                Eigen::Vector3f n_obs_2(...);
//                Eigen::Vector3f n_1 = sensor_poses[sensor1].block(0,0,3,3) * n_obs_1;
//                Eigen::Vector3f n_2 = sensor_poses[sensor2].block(0,0,3,3) * n_obs_2;
//                Eigen::Vector3f n_diff = (n_1 - n_2);
//                sum_squared_error += n_diff.dot(n_diff);
//            }
        }
    }
    return sum_squared_error;
}
