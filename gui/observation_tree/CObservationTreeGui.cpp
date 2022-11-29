#include "CObservationTreeGui.h"

#include <QProgressDialog>

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/CTicTac.h>

using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::obs;
using namespace mrpt::system;

CObservationTreeGui::CObservationTreeGui(const std::string &rawlog_filename, const mrpt::config::CConfigFile &config_file, QObject *parent) :
    QAbstractItemModel(parent),
    CObservationTree(rawlog_filename, config_file)
{
}

CObservationTreeGui::~CObservationTreeGui()
{
}

void CObservationTreeGui::addTextObserver(CTextObserver *observer)
{
	m_text_observers.push_back(observer);
}

void CObservationTreeGui::publishText(const std::string &msg)
{
	for(CTextObserver *observer : m_text_observers)
	{
		observer->onReceivingText(msg);
	}
}

QModelIndex CObservationTreeGui::index(int row, int column, const QModelIndex &parent) const
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

QModelIndex CObservationTreeGui::parent(const QModelIndex &index) const
{
	if(!index.isValid())
		return QModelIndex();

	CObservationTreeItem *child_item = static_cast<CObservationTreeItem*>(index.internalPointer());
	CObservationTreeItem *parent_item = child_item->parentItem();

	if(parent_item == m_rootitem)
		return QModelIndex();

	return createIndex(parent_item->row(), 0, parent_item);
}

int CObservationTreeGui::rowCount(const QModelIndex &parent) const
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

int CObservationTreeGui::columnCount(const QModelIndex &parent) const
{
	Q_UNUSED(parent);
	return 1;
}

QVariant CObservationTreeGui::data(const QModelIndex &index, int role) const
{
	if(!index.isValid())
		return QVariant();
	
	if(role != Qt::DisplayRole)
		return QVariant();

	CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());

	return QString::fromStdString(item->itemId());
}

Qt::ItemFlags CObservationTreeGui::flags(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	return QAbstractItemModel::flags(index);
}

CObservationTreeItem *CObservationTreeGui::getItem(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	CObservationTreeItem *item = static_cast<CObservationTreeItem*>(index.internalPointer());
	return item;
}
