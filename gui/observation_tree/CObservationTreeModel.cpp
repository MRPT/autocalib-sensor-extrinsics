#include <observation_tree/CObservationTreeModel.h>

#include <QProgressDialog>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/CTicTac.h>

using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::obs;
using namespace mrpt::system;

CObservationTreeModel::CObservationTreeModel(QObject *parent)
    : QAbstractItemModel(parent)
{
	m_rootitem = new CObservationTreeItem(QString("root"));
}

CObservationTreeModel::~CObservationTreeModel()
{
	delete m_rootitem;
}

void CObservationTreeModel::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CObservationTreeModel::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

void CObservationTreeModel::loadModel(const std::string &rawlog_filename)
{
	m_rawlog_filename = rawlog_filename;

	CTicTac stop_watch;
	double time_to_load;
	stop_watch.Tic();

	CFileGZInputStream rawlog(m_rawlog_filename);
	CSerializable::Ptr obj;

	bool read = true;
	size_t obs_count = 0;
	QString sensor_label, obs_label;

	uint64_t rlog_size = rawlog.getTotalBytesCount() >> 10;
	QProgressDialog progress_dialog("Loading rawlog file..", "Abort", 0, rlog_size);
	progress_dialog.setWindowTitle("Load Progress");
	progress_dialog.setWindowModality(Qt::WindowModal);
	progress_dialog.setMinimumDuration(0);

	while(read)
	{
		try
		{
			archiveFrom(rawlog) >> obj;

			CObservation::Ptr obs = std::dynamic_pointer_cast<CObservation>(obj);
			obs_count++;

			sensor_label = QString::fromStdString(obs->sensorLabel);
			obs_label = QString::fromStdString(obs->sensorLabel + " : " + (obs->GetRuntimeClass()->className));

			if(obs_count == 1)
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
					m_count_of_label.push_back(0);
				}

				m_count_of_label[m_sensor_labels.indexOf(*iter)]++;
			}


			m_rootitem->appendChild(new CObservationTreeItem(QString("[#" + QString::number(obs_count - 1) + "] " + obs_label), obs, m_rootitem));

			if(progress_dialog.wasCanceled())
			{
				read = false;
				delete m_rootitem;
				m_rootitem = nullptr;
			}

			progress_dialog.setValue(rawlog.getPosition() >> 10);
		}

		catch(std::exception &e)
		{
			read = false;
			time_to_load = stop_watch.Tac();
			publishText("Rawlog loaded. Click on any observation item on the left to visualize it.");

			std::string stats_string;
			stats_string = "RAWLOG STATS:";
			stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";
			stats_string += "\nTime elapsed: " + std::to_string(time_to_load) + " seconds";
			stats_string += "\nNumber of observations loaded: " + std::to_string(obs_count);
			stats_string += "\nNumber of unique sensors found in rawlog: " + std::to_string(m_sensor_labels.count());
			stats_string += "\n\nSummary of sensors found in rawlog:";
			stats_string += "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - ";

			for(size_t i = 0; i < m_sensor_labels.count(); i++)
			{
				stats_string += "\nSensor #" + std::to_string(i);
				stats_string += "\nSensor label : Class :: " + m_sensor_labels[i].toStdString();
				stats_string += "\nNumber of observations: " + std::to_string(m_count_of_label[i]) + "\n";
			}

			publishText(stats_string);
		}
	}

	progress_dialog.setValue(rlog_size);
}

void CObservationTreeModel::syncObservations(const QStringList &selected_sensor_labels, const int &max_delay, std::vector<std::vector<int>> &sync_obs_indices)
{
	CObservationTreeItem *new_rootitem = new CObservationTreeItem(QString("root"));
	CObservation::Ptr curr_obs, next_obs;
	QString curr_sensor_label, obs_label;
	QStringList sensor_labels_in_set;
	size_t obs_sets_count = 0;
	std::pair<CObservation::Ptr, int> curr_obs_with_model_id;
	std::vector<std::pair<CObservation::Ptr, int>> obs_set;
	mrpt::system::TTimeStamp curr_ts, next_ts, set_ts;
	double delay;

	std::vector<std::vector<int>> sync_obs_indices_tmp;

	sync_obs_indices_tmp = sync_obs_indices;

	for(size_t i = 0; i < m_rootitem->childCount(); i++)
	{
		curr_obs = std::dynamic_pointer_cast<CObservation3DRangeScan>(m_rootitem->child(i)->getObservation());
		curr_sensor_label = QString::fromStdString(curr_obs->sensorLabel);
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
						new_rootitem->appendChild(new CObservationTreeItem(QString("Observations set #" + QString::number(obs_sets_count++)), 0, new_rootitem));

						auto iter2 = obs_set.begin();
						for(iter = sensor_labels_in_set.begin(); (iter < sensor_labels_in_set.end()) && (iter2 < obs_set.end()); iter++, iter2++)
						{
							obs_label = QString::fromStdString("[#" + std::to_string((*iter2).second) + "] "
							                                   + (*iter).toStdString() + " : " + (*iter2).first->GetRuntimeClass()->className);
							new_rootitem->child(new_rootitem->childCount() - 1)->appendChild(new CObservationTreeItem(obs_label, (*iter2).first,
							                                                                                          new_rootitem->child(new_rootitem->childCount() - 1), (*iter2).second));
							sync_obs_indices_tmp[selected_sensor_labels.indexOf(*iter)].push_back((*iter2).second);
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
		new_rootitem->appendChild(new CObservationTreeItem(QString("Observations set #" + QString::number(obs_sets_count++)), 0, new_rootitem));

		auto iter2 = obs_set.begin();
		for(auto iter = sensor_labels_in_set.begin(); (iter < sensor_labels_in_set.end()) && (iter2 < obs_set.end()); iter++, iter2++)
		{
			obs_label = QString::fromStdString("[#" + std::to_string((*iter2).second) + "] "
			                                   + (*iter).toStdString() + " : " + (*iter2).first->GetRuntimeClass()->className);
			new_rootitem->child(new_rootitem->childCount() - 1)->appendChild(new CObservationTreeItem(obs_label, (*iter2).first,
			                                                                                          new_rootitem->child(new_rootitem->childCount() - 1), (*iter2).second));
			sync_obs_indices_tmp[selected_sensor_labels.indexOf(*iter)].push_back((*iter2).second);
		}

		sensor_labels_in_set.clear();
		obs_set.clear();
	}

	// removing any duplicate entries
	for(size_t i = 0; i < sync_obs_indices_tmp.size(); i++)
	{
		for(size_t j = 0; j < sync_obs_indices_tmp[i].size(); j++)
		{
			if(j > 0)
			{
				auto iter = std::find(sync_obs_indices[i].begin(), sync_obs_indices[i].end(), sync_obs_indices_tmp[i][j]);
				if(iter == sync_obs_indices[i].end())
					sync_obs_indices[i].push_back(sync_obs_indices_tmp[i][j]);
			}

			else
				sync_obs_indices[i].push_back(sync_obs_indices_tmp[i][j]);
		}
	}

	m_rootitem = new_rootitem;
	m_sensor_labels = selected_sensor_labels;
}

QModelIndex CObservationTreeModel::index(int row, int column, const QModelIndex &parent) const
{
	if(!hasIndex(row, column, parent))
		return QModelIndex();

	CObservationTreeItem *parent_item;

	if(!parent.isValid())
		parent_item =  m_rootitem;
	else
		parent_item = static_cast<CObservationTreeItem*>(parent.internalPointer());

	CObservationTreeItem *child_item = parent_item->child(row);

	if(child_item)
		return createIndex(row, column, child_item);
	else
		return QModelIndex();
}

QModelIndex CObservationTreeModel::parent(const QModelIndex &index) const
{
	if(!index.isValid())
		return QModelIndex();

	CObservationTreeItem *child_item = static_cast<CObservationTreeItem*>(index.internalPointer());
	CObservationTreeItem *parent_item = child_item->parentItem();

	if(parent_item == m_rootitem)
		return QModelIndex();

	return createIndex(parent_item->row(), 0, parent_item);
}

int CObservationTreeModel::rowCount(const QModelIndex &parent) const
{
	CObservationTreeItem *parent_item;

	if(parent.column() > 0)
		return 0;

	if(!parent.isValid())
		parent_item = m_rootitem;
	else
		parent_item = static_cast<CObservationTreeItem*>(parent.internalPointer());

	return parent_item->childCount();
}

int CObservationTreeModel::columnCount(const QModelIndex &parent) const
{
	Q_UNUSED(parent);
	return 1;
}

QVariant CObservationTreeModel::data(const QModelIndex &index, int role) const
{
	if(!index.isValid())
		return QVariant();
	
	if(role != Qt::DisplayRole)
		return QVariant();

	CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

	return item->displayData();
}

CObservation::Ptr CObservationTreeModel::observationData(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());
	return item->getObservation();
}

Qt::ItemFlags CObservationTreeModel::flags(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	return QAbstractItemModel::flags(index);
}

CObservationTreeItem *CObservationTreeModel::getRootItem() const
{
	return this->m_rootitem;
}

CObservationTreeItem *CObservationTreeModel::getItem(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());
	return item;
}

QStringList CObservationTreeModel::getSensorLabels() const
{
	return this->m_sensor_labels;
}
