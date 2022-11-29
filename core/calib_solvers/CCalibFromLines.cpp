#include "CCalibFromLines.h"
#include <mrpt/math/geometry.h>

CCalibFromLines::CCalibFromLines(CObservationTree *model, TCalibFromLinesParams *params) :
    CExtrinsicCalib(model)
{
	this->params = params;
}

CCalibFromLines::~CCalibFromLines(){}

void CCalibFromLines::segmentLines(const cv::Mat &image, Eigen::MatrixXf &range, const mrpt::img::TCamera &camera_params, const Eigen::Affine3f &intensity_to_depth_transform, std::vector<CLine> &lines)
{
	cv::Mat canny_image;

	cv::Canny(image, canny_image, params->seg.clow_threshold, params->seg.clow_threshold * params->seg.chigh_to_low_ratio, params->seg.ckernel_size);

	std::vector<cv::Vec2f> hlines;

	cv::HoughLines(canny_image, hlines, 1, CV_PI, params->seg.hthreshold);


	double rho, theta;
	double cos_theta, sin_theta;
	double m, c, c_max;
	CLine line;

	int x, y, xf, yf;

	for(size_t n(0); n < hlines.size(); n++)
	{
		if(hlines[n][0] < 0)
		{
			rho = -hlines[n][0];
			theta = hlines[n][1] - CV_PI;
		}

		else
		{
			rho = hlines[n][0];
			theta = hlines[n][1];
		}

		if (rho == 0 && theta == 0)
			continue;

		if (fabs(theta) < 0.00001)
		{
			x = xf = static_cast<int>(rho + 0.5);
			y = 0;
			yf = canny_image.rows - 1;
		}

		else
		{
			cos_theta = cos(theta);
			sin_theta = sin(theta);
			m = -cos_theta / sin_theta;
			c = rho * (sin_theta - m * cos_theta);

			if (c >= 0)
			{
				if (c < canny_image.rows)
				{
					x = 0;
					y = static_cast<int>(c);
				}
				else
				{
					y = canny_image.rows - 1;
					x = static_cast<int>((y - c) / m);
				}
			}

			else
			{
				x = static_cast<int>(-c / m);
				y = 0;
			}

			c_max = m * (canny_image.cols - 1) + c;
			if (c_max >= 0)
			{
				if (c_max < canny_image.rows)
				{
					xf = canny_image.cols - 1;
					yf = static_cast<int>(c_max);
				}

				else
				{
					yf = canny_image.rows - 1;
					xf = static_cast<int>((yf - c) / m);
				}
			}
			else
			{
				xf = static_cast<int>(-c / m);
				yf = 0;
			}
		}

		line.rho = rho;
		line.theta = theta;
		line.m = m;
		line.c = c;

		// Bresenham algorithm

		bool onSegment = false;
		int memory;
		int memoryX = 0, memoryY = 0;
		int xPrev = 0, yPrev = 0;
		size_t nbPixels = 0;

		int w = xf - x;
		int h = yf - y;
		int dx1, dy1, dx2, dy2 = 0;

		int longest, shortest;
		int numerator;

		if (w < 0)
		{
			longest = -w;
			dx1 = -1;
			dx2 = -1;
		}
		else
		{
			longest = w;
			dx1 = 1;
			dx2 = 1;
		}

		if (h < 0)
		{
			shortest = -h;
			dy1 = -1;
		}

		else
		{
			shortest = h;
			dy1 = 1;
		}

		if (longest <= shortest)
		{
			memory = longest;
			longest = shortest;
			shortest = memory;
			dx2 = 0;
			if (h < 0)
			{
				dy2 = -1;
			}

			else
			{
				dy2 = 1;
			}
		}

		numerator = longest / 2;

		for (int i(0); i <= longest; ++i)
		{
			if (onSegment)
			{
				if (canny_image.at<char>(y, x) == 0 || i == longest)
				{
					onSegment = false;
					if (nbPixels >= params->seg.hthreshold)
					{
						std::array<Eigen::Vector2i, 2> end_points;
						end_points[0] = Eigen::Vector2i(memoryX, memoryY);
						end_points[1] = Eigen::Vector2i(xPrev, yPrev);

						Eigen::Vector2i mean_point = Eigen::Vector2i((memoryX + xPrev)/2, (memoryY + yPrev)/2);

						Eigen::Vector3f ray;
						ray[0] = (mean_point[0] - camera_params.cx())/camera_params.fx();
						ray[1] = (mean_point[1] - camera_params.cy())/camera_params.fy();
						ray[2] = 1;

						Eigen::Vector2i l_temp = (end_points[0] - end_points[1]);
						Eigen::Vector3i l(l_temp[0], l_temp[1], 0);

						Eigen::Vector3f normal;
						mrpt::math::crossProduct3D(l, ray, normal);

						Eigen::Vector3f p;
						utils::backprojectTo3D(mean_point, range, camera_params, p);

						std::array<Eigen::Vector3f, 2> end_points3D;
						utils::backprojectTo3D(end_points[0], range, camera_params, end_points3D[0]);
						utils::backprojectTo3D(end_points[1], range, camera_params, end_points3D[1]);

						Eigen::Vector3f v;
						v = end_points3D[1] - end_points3D[0];

						line.end_points = end_points;
						line.end_points3D[0] = intensity_to_depth_transform * end_points3D[0];
						line.end_points3D[1] = intensity_to_depth_transform * end_points3D[1];
						line.mean_point = mean_point;
						line.ray = intensity_to_depth_transform * ray;
						line.l = l;
						line.normal = intensity_to_depth_transform * normal;
						line.p = intensity_to_depth_transform * p;
						line.v = intensity_to_depth_transform * v;
						lines.push_back(line);
					}
				}

				else
				{
					++nbPixels;
				}
			}

			else if (canny_image.at<char>(y, x) != 0)
			{
				onSegment = true;
				nbPixels = 0;
				memoryX = x;
				memoryY = y;
			}

			xPrev = x;
			yPrev = y;

			numerator += shortest;

			if(numerator >= longest)
			{
				numerator -= longest;
				x += dx1;
				y += dy1;
			}

			else
			{
				x += dx2;
				y += dy2;
			}
		}
	}
}

void CCalibFromLines::findPotentialMatches(const std::vector<std::vector<CLine>> &lines, const int &set_id)
{
	std::vector<Eigen::Vector2f> sensor_pose_uncertainties = sync_model->getSensorUncertainties();

	for(int i = 0; i < lines.size()-1; ++i)
		for(int j = i+1; j < lines.size(); ++j)
		{
			for(int ii = 0; ii < lines[i].size(); ++ii)
			{
				for(int jj = 0; jj < lines[j].size(); ++jj)
				{
					Eigen::Vector3f n_ii = sync_model->getSensorPoses()[i].block(0,0,3,3) * lines[i][ii].normal;
					Eigen::Vector3f v_ii = sync_model->getSensorPoses()[i].block(0,0,3,3) * lines[i][ii].v;

					Eigen::Vector3f n_jj = sync_model->getSensorPoses()[j].block(0,0,3,3) * lines[j][jj].normal;
					Eigen::Vector3f v_jj = sync_model->getSensorPoses()[j].block(0,0,3,3) * lines[j][jj].v;

					if((n_ii.dot(n_jj) > cos(sensor_pose_uncertainties[j](0) * (M_PI/180))) && (n_ii.dot(v_jj) < cos((90 - sensor_pose_uncertainties[j](0)) * (M_PI/180))))
					{
						std::array<int,3> potential_match{set_id, ii, jj};
						mmv_line_corresp[i][j].push_back(potential_match);
					}
				}
			}

//			for stats printing when no matches exist
//			if((lines[i].size() == 0) || (lines[j].size() == 0))
//			{
//				std::array<int,3> temp_match{-1, -1, -1};
//				mmv_line_corresp[i][j].push_back(temp_match);
//			}
		}
}

Scalar CCalibFromLines::computeRotationResidual()
{
}

Scalar CCalibFromLines::computeRotation()
{
}

Scalar CCalibFromLines::computeTranslation()
{
}

