#include <core_gui/CCalibFromLinesGui.h>

#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace mrpt::obs;

CCalibFromLinesGui::CCalibFromLinesGui(CObservationTree *model, TCalibFromLinesParams *params) :
    CCalibFromLines(model)
{
	m_params = params;
}

CCalibFromLinesGui::~CCalibFromLinesGui()
{
}

void CCalibFromLinesGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CCalibFromLinesGui::addLinesObserver(CLinesObserver *observer)
{
	m_lines_observers.push_back(observer);
}

void CCalibFromLinesGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CCalibFromLinesGui::publishLines(const int &sensor_id, const int &sync_obs_id)
{
	for(CLinesObserver *observer : m_lines_observers)
	{
		observer->onReceivingLines(sensor_id, mvv_lines[sensor_id][sync_obs_id]);
	}
}

CalibrationFromLinesStatus CCalibFromLinesGui::calibStatus()
{
	return m_params->calib_status;
}

void CCalibFromLinesGui::extractLines()
{
	publishText("****Running line segmentation algorithm****");

	CObservationTreeItem *root_item, *tree_item, *item;
	CObservation3DRangeScan::Ptr obs_item;
	size_t sync_obs_id = 0;
	root_item = sync_model->getRootItem();

//	T3DPointsProjectionParams projection_params;
//	projection_params.MAKE_DENSE = false;
//	projection_params.MAKE_ORGANIZED = true;

//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<cv::Vec4i> segmented_lines;
	size_t n_lines;
	double line_segment_start, line_segment_end;

	std::vector<std::string> selected_sensor_labels;
	std::string prev_sensor_label;
	mrpt::system::TTimeStamp prev_ts = 0;

	selected_sensor_labels = sync_model->getSensorLabels();

	for(size_t i = 0; i < selected_sensor_labels.size(); i++)
	{
		publishText("**Extracting lines from sensor #" + std::to_string(i) + " observations**");
		mvv_lines[i].resize((sync_model->getSyncIndices()[i]).size());

		//let's run it for 15 sets
		for(size_t j = 0; j < 15; j++)
		{
			tree_item = root_item->child(j);

			for(size_t k = 0; k < tree_item->childCount(); k++)
			{
				item = tree_item->child(k);
				obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(item->getObservation());
				if((obs_item->sensorLabel == selected_sensor_labels[i]) && (obs_item->timestamp != prev_ts))
				{
//					if(item->cloud() != nullptr)
//						cloud  = item->cloud();

//					else
//					{
//						obs_item->project3DPointsFromDepthImageInto(*cloud, projection_params);
//						cloud->is_dense = false;
//						item->cloud() = cloud;
//					}

					cv::Mat image = cv::cvarrToMat(obs_item->intensityImage.getAs<IplImage>());

					line_segment_start = pcl::getTime();
					segmented_lines.clear();
					segmentLines(image, m_params->seg, segmented_lines);
					line_segment_end = pcl::getTime();

					n_lines = segmented_lines.size();
					publishText(std::to_string(n_lines) + " line(s) extracted from observation #" + std::to_string(tree_item->child(k)->getPriorIndex())
					            + "\nTime elapsed: " +  std::to_string(line_segment_end - line_segment_start));

					mvv_lines[i][sync_obs_id] = segmented_lines;
					sync_obs_id++;
					prev_ts = obs_item->timestamp;
				}
			}
		}

		sync_obs_id = 0;
	}

	m_params->calib_status = CalibrationFromLinesStatus::LINES_EXTRACTED;
}

void CCalibFromLinesGui::matchLines()
{

}
