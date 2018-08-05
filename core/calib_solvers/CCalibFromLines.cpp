#include "CCalibFromLines.h"
#include <mrpt/math/geometry.h>

CCalibFromLines::CCalibFromLines(CObservationTree *model) : CExtrinsicCalib(model)
{}

CCalibFromLines::~CCalibFromLines(){}

void CCalibFromLines::segmentLines(const cv::Mat &image, mrpt::math::CMatrix &range, const TLineSegmentationParams &params, const mrpt::img::TCamera &camera_params, std::vector<CLine> &lines)
{
	cv::Mat canny_image;

	cv::Canny(image, canny_image, params.clow_threshold, params.clow_threshold * params.chigh_to_low_ratio, params.ckernel_size);

	std::vector<cv::Vec2f> hlines;

	cv::HoughLines(canny_image, hlines, 1, CV_PI, params.hthreshold);


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
					if (nbPixels >= params.hthreshold)
					{
						std::array<cv::Vec2i, 2> end_points;
						end_points[0] = cv::Vec2i(memoryX, memoryY);
						end_points[1] = cv::Vec2i(xPrev, yPrev);

						cv::Vec2i mean_point = cv::Vec2i((memoryX + xPrev)/2, (memoryY + yPrev)/2);

						cv::Vec3d ray;
						ray[0] = (mean_point[0] - camera_params.cx())/camera_params.fx();
						ray[1] = (mean_point[1] - camera_params.cy())/camera_params.fy();
						ray[2] = 1;

						cv::Vec3d l, normal;
						cv::Vec2d l_temp = end_points[0] - end_points[1];
						l[0] = l_temp[0]; l[1] = l_temp[1]; l[2]= 0;
						mrpt::math::crossProduct3D(l, ray, normal);

						cv::Vec3d p;
						utils::backprojectTo3D(mean_point, range, camera_params, p);

						cv::Vec2i end_point1, end_point2;
						std::array<cv::Vec3d, 2> end_points3D;

						utils::backprojectTo3D(end_points[0], range, camera_params, end_points3D[0]);
						utils::backprojectTo3D(end_points[1], range, camera_params, end_points3D[1]);

						cv::Vec3d v;
						v = end_points3D[1] - end_points3D[0];

						line.end_points = end_points;
						line.end_points3D = end_points3D;
						line.mean_point = mean_point;
						line.ray = ray;
						line.l = l;
						line.normal = normal;
						line.p = p;
						line.v = v;
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

Scalar CCalibFromLines::computeRotCalibResidual(const std::vector<Eigen::Matrix4f> &sensor_poses)
{

}

Scalar CCalibFromLines::computeRotCalibration(const TSolverParams &params, const std::vector<Eigen::Matrix4f> & sensor_poses, std::string &stats)
{

}
