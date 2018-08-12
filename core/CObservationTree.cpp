#include "CObservationTree.h"

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/rtti/CObject.h>

using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::obs;

CObservationTree::CObservationTree(const std::string &rawlog_path, const mrpt::config::CConfigFile &config_file)
{
	m_rootitem = new CObservationTreeItem("root");
	m_rawlog_path = rawlog_path;
	m_config_file = config_file;
	m_synced = false;
}

CObservationTree::~CObservationTree()
{
	delete m_rootitem;
}

void CObservationTree::loadTree()
{	
	CFileGZInputStream rawlog(m_rawlog_path);
	CSerializable::Ptr obj;

	bool read = true;
	std::string sensor_label, obs_label;

	while(read)
	{
		try
		{
			archiveFrom(rawlog) >> obj;

			CObservation::Ptr obs = std::dynamic_pointer_cast<CObservation>(obj);
			m_obs_count++;

			sensor_label = obs->sensorLabel;
			obs_label = sensor_label + " : " + (obs->GetRuntimeClass()->className);

			if(m_obs_count == 1)
			{
				m_sensor_labels.push_back(sensor_label);
				m_count_of_label.push_back(1);
			}

			else
			{
				auto iter = std::find(m_sensor_labels.begin(), m_sensor_labels.end(), sensor_label);
				if(iter == m_sensor_labels.end())
				{
					m_sensor_labels.push_back(sensor_label);
					m_count_of_label.push_back(1);
				}

				else
					m_count_of_label[utils::findItemIndexIn(m_sensor_labels, sensor_label)]++;
			}

			m_rootitem->appendChild(new CObservationTreeItem("[#" + std::to_string(m_obs_count - 1) + "] " + obs_label, obs, m_rootitem));
		}

		catch(std::exception &e)
		{
			read = false;
		}
	}

	Eigen::Matrix4f rt;
	Eigen::Vector2f uncertain;
	for(size_t i = 0; i < m_sensor_labels.size(); i++)
	{
		m_config_file.read_matrix("initial_calibration", m_sensor_labels[i], rt, Eigen::Matrix4f(), true);
		m_config_file.read_vector("initial_uncertainty", m_sensor_labels[i], Eigen::Vector2f(), uncertain, true);
		m_sensor_poses.push_back(rt);
		m_sensor_pose_uncertainties.push_back(uncertain);
	}
}

void CObservationTree::syncObservations(const std::vector<std::string> &selected_sensor_labels, const int &max_delay)
{
	CObservationTreeItem *new_rootitem = new CObservationTreeItem("root");
	CObservation::Ptr curr_obs, next_obs;
	std::string curr_sensor_label, obs_label;
	std::vector<std::string> sensor_labels_in_set;
	size_t obs_sets_count = 0;
	std::pair<CObservation::Ptr, int> curr_obs_with_model_id;
	std::vector<std::pair<CObservation::Ptr, int>> obs_set;
	mrpt::system::TTimeStamp curr_ts, next_ts, set_ts;
	double delay;

	std::vector<std::vector<int>> sync_indices_tmp;

	sync_indices_tmp.resize(selected_sensor_labels.size());
	m_sync_indices.resize(selected_sensor_labels.size());

	for(size_t i = 0; i < m_rootitem->childCount(); i++)
	{
		curr_obs = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_rootitem->child(i)->getObservation());
		curr_sensor_label = curr_obs->sensorLabel;
		curr_ts = curr_obs->timestamp;
		curr_obs_with_model_id.first = curr_obs;
		curr_obs_with_model_id.second = i;

		auto iter = std::find(selected_sensor_labels.begin(), selected_sensor_labels.end(), curr_sensor_label);

		if(iter != selected_sensor_labels.end())
		{
			if(obs_set.size() == 0)
			{
				sensor_labels_in_set.push_back(curr_sensor_label);
				obs_set.push_back(curr_obs_with_model_id);
				set_ts = curr_ts;
				continue;
			}

			else
			{
				iter = std::find(sensor_labels_in_set.begin(), sensor_labels_in_set.end(), curr_sensor_label);
				delay = mrpt::system::timeDifference(set_ts, curr_ts);
				if(iter == sensor_labels_in_set.end() && (delay <= max_delay))
				{
					sensor_labels_in_set.push_back(curr_sensor_label);
					obs_set.push_back(curr_obs_with_model_id);
				}

				else
				{
					if(sensor_labels_in_set.size() == selected_sensor_labels.size())
					{
						new_rootitem->appendChild(new CObservationTreeItem("Observations set #" + std::to_string(obs_sets_count++), 0, new_rootitem));

						auto iter2 = obs_set.begin();
						for(iter = sensor_labels_in_set.begin(); (iter < sensor_labels_in_set.end()) && (iter2 < obs_set.end()); iter++, iter2++)
						{
							obs_label = "[#" + std::to_string((*iter2).second) + "] "
							                                   + (*iter) + " : " + (*iter2).first->GetRuntimeClass()->className;
							new_rootitem->child(new_rootitem->childCount() - 1)->appendChild(new CObservationTreeItem(obs_label, (*iter2).first,
							                                                                                          new_rootitem->child(new_rootitem->childCount() - 1), (*iter2).second));
							sync_indices_tmp[utils::findItemIndexIn(selected_sensor_labels, *iter)].push_back((*iter2).second);
						}
					}

					i -= sensor_labels_in_set.size();
					sensor_labels_in_set.clear();
					obs_set.clear();
				}
			}
		}
	}

	// inserting left over set
	if(obs_set.size() > 0 && sensor_labels_in_set.size() == selected_sensor_labels.size())
	{
		new_rootitem->appendChild(new CObservationTreeItem("Observations set #" + std::to_string(obs_sets_count++), 0, new_rootitem));

		auto iter2 = obs_set.begin();
		for(auto iter = sensor_labels_in_set.begin(); (iter < sensor_labels_in_set.end()) && (iter2 < obs_set.end()); iter++, iter2++)
		{
			obs_label = "[#" + std::to_string((*iter2).second) + "] "
			                                   + (*iter) + " : " + (*iter2).first->GetRuntimeClass()->className;
			new_rootitem->child(new_rootitem->childCount() - 1)->appendChild(new CObservationTreeItem(obs_label, (*iter2).first,
			                                                                                          new_rootitem->child(new_rootitem->childCount() - 1), (*iter2).second));
			sync_indices_tmp[utils::findItemIndexIn(selected_sensor_labels, *iter)].push_back((*iter2).second);
		}

		sensor_labels_in_set.clear();
		obs_set.clear();
	}

	// removing any duplicate entries
	for(size_t i = 0; i < sync_indices_tmp.size(); i++)
	{
		for(size_t j = 0; j < sync_indices_tmp[i].size(); j++)
		{
			if(j > 0)
			{
				auto iter = std::find(m_sync_indices[i].begin(), m_sync_indices[i].end(), sync_indices_tmp[i][j]);
				if(iter == m_sync_indices[i].end())
					m_sync_indices[i].push_back(sync_indices_tmp[i][j]);
			}

			else
				m_sync_indices[i].push_back(sync_indices_tmp[i][j]);
		}
	}

	m_rootitem = new_rootitem;
	m_sensor_labels = selected_sensor_labels;
	m_synced = true;
	m_sync_offset = max_delay;

	Eigen::Matrix4f rt;
	Eigen::Vector2f uncertain;
	for(size_t i = 0; i < m_sensor_labels.size(); i++)
	{
		m_config_file.read_matrix("initial_calibration", m_sensor_labels[i], rt, Eigen::Matrix4f(), true);
		m_config_file.read_vector("initial_uncertainty", m_sensor_labels[i], Eigen::Vector2f(), uncertain, true);
		m_sensor_poses.push_back(rt);
		m_sensor_pose_uncertainties.push_back(uncertain);
	}
}

std::string CObservationTree::getRawlogPath() const
{
	return this->m_rawlog_path;
}

CObservationTreeItem *CObservationTree::getRootItem() const
{
	return this->m_rootitem;
}

int CObservationTree::getObsCount() const
{
	return this->m_obs_count;
}

int CObservationTree::getNumberOfSensors() const
{
	return this->m_sensor_labels.size();
}

std::vector<std::string> CObservationTree::getSensorLabels() const
{
	return this->m_sensor_labels;
}

std::vector<int> CObservationTree::getCountOfLabel() const
{
	return this->m_count_of_label;
}

std::vector<Eigen::Matrix4f> CObservationTree::getSensorPoses() const
{
	return this->m_sensor_poses;
}

std::vector<Eigen::Vector2f> CObservationTree::getSensorUncertainties() const
{
	return this->m_sensor_pose_uncertainties;
}

void CObservationTree::setSensorPose(const Eigen::Matrix4f &sensor_pose, const int &sensor_index)
{
	this->m_sensor_poses[sensor_index] = sensor_pose;
}

bool CObservationTree::setSensorPoses(const std::vector<Eigen::Matrix4f> &sensor_poses)
{
	if(sensor_poses.size() != this->m_sensor_labels.size())
		return 0;
	else
		this->m_sensor_poses = sensor_poses;
}

void CObservationTree::setSensorUncertainty(const Eigen::Vector2f &sensor_uncertainty, const int &sensor_index)
{
	this->m_sensor_pose_uncertainties[sensor_index] = sensor_uncertainty;
}

bool CObservationTree::setSensorUncertainties(const std::vector<Eigen::Vector2f> &uncertainties)
{
	if(uncertainties.size() != this->m_sensor_pose_uncertainties.size())
		return 0;
	else
		this->m_sensor_pose_uncertainties = uncertainties;
}

std::vector<std::vector<int>> CObservationTree::getSyncIndices() const
{
	return this->m_sync_indices;
}

int CObservationTree::findSyncIndexFromSet(const int &set_id, const std::string &sensor_label) const
{
	if(!m_synced)
		return -1;

	CObservationTreeItem *item = m_rootitem->child(set_id);
	size_t sensor_index = utils::findItemIndexIn(m_sensor_labels, sensor_label);

	for(size_t i = 0; i < item->childCount(); i++)
	{
		if(item->child(i)->getObservation()->sensorLabel == sensor_label)
		{
			return utils::findItemIndexIn(m_sync_indices[sensor_index], item->child(i)->getPriorIndex());
		}
	}
}
