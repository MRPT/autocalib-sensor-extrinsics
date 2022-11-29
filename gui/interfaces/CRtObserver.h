#pragma once

#include <Eigen/Core>

/**
 * @brief Observer (listener) that receives the estimated relative transformations from the GUI calib wrapper classes.
 */

class CRtObserver
{
    public:
	    virtual void ontReceivingRt(const std::vector<Eigen::Matrix4f> &relative_transformations) = 0;
};
