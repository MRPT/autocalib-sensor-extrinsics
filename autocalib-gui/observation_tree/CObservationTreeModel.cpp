#include "CObservationTreeModel.h"

CObservationTreeModel::CObservationTreeModel(const mrpt::obs::CRawlog &rawlog, QObject *parent = 0)
	: QAbstractItemModel(parent)
{
	setupModelData();
}

CObservationTreeModel::~CObservationTreeModel()
{
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
	if(!hasIndex(row,column,index))
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
		parent_item = static_const<CObservationTreeItem*>(parent.internalPointer());

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

	return item->data(index.column());
}

Qt::ItemFlags CObservationTreeModel::flags(const QModelIndex &index) const
{
	if(!index.isValid())
		return 0;

	return QAbstractItemModel::flags(index);
}
