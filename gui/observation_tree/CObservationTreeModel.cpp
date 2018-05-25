#include "CObservationTreeModel.h"

#include <QDebug>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/rtti/CObject.h>

using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::obs;

CObservationTreeModel::CObservationTreeModel(const std::string &rawlog_filename, QObject *parent)
	: QAbstractItemModel(parent)
{
	m_rootitem = new CObservationTreeItem(QString("root"));

	CFileGZInputStream rawlog(rawlog_filename);
	CSerializable::Ptr obj;

	bool read = true;
	uint obs_count = 0, obs_sets_count = 0;
	QStringList obs_labels, obs_labels_in_set;
	std::vector<CObservation::Ptr> obs_set;
	QString obs_label;

	while(read)
	{
		try
		{
			archiveFrom(rawlog) >> obj;

			CObservation::Ptr obs = std::dynamic_pointer_cast<CObservation>(obj);
			obs_count++;

			obs_label = QString::fromStdString(obs->sensorLabel + " : " + (obs->GetRuntimeClass()->className));

			if(obs_count == 1)
			{
				obs_labels.push_back(obs_label);
				obs_labels_in_set.push_back(obs_label);
				obs_set.push_back(obs);
				continue;
			}

			else
			{
				auto iter = std::find(obs_labels.begin(), obs_labels.end(), obs_label);
				if(iter == obs_labels.end())
					obs_labels.push_back(obs_label);

				iter = std::find(obs_labels_in_set.begin(), obs_labels_in_set.end(), obs_label);
				if(iter == obs_labels_in_set.end())
				{
					obs_labels_in_set.push_back(obs_label);
					obs_set.push_back(obs);
				}

				else
				{
					m_rootitem->appendChild(new CObservationTreeItem(QString("Observations set #" + QString::number(obs_sets_count++)), 0, m_rootitem));

					auto iter2 = obs_set.begin();
					for(iter = obs_labels_in_set.begin(); iter < obs_labels_in_set.end(), iter2 < obs_set.end(); iter++, iter2++)
					{
						m_rootitem->child(m_rootitem->childCount() - 1)->appendChild(new CObservationTreeItem(*iter, *iter2, m_rootitem->child(m_rootitem->childCount() - 1)));
					}

					obs_labels_in_set.clear();
					obs_set.clear();
					obs_labels_in_set.push_back(obs_label);
					obs_set.push_back(obs);
				}

			}

		}

		catch(std::exception &e)
		{
			read = false;
		}
	}
}

CObservationTreeModel::~CObservationTreeModel()
{
	delete m_rootitem;
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

	return item->observationData();
}

Qt::ItemFlags CObservationTreeModel::flags(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	return QAbstractItemModel::flags(index);
}
