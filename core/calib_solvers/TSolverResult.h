#pragma once

#include <vector>
#include <Eigen/Core>

/** Structure meant to hold the results of the calibration solver, useful for printing results. */

struct TSolverResult
{
	/** The number of iterations elapsed. */
	int num_iters;

	/** The error during initialization. */
	float init_error;

	/** The error with the estimated parameters. */
	float final_error;

	/** The estimated parameters. */
	std::vector<Eigen::Matrix4f> estimate;

	/** Result message. */
	std::string msg;
};
