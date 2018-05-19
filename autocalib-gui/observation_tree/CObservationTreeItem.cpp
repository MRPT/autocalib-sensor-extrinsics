#include "CObservationTreeItem.h"
#include <QStringList>

CObservationTreeItem::CObservationTreeItem(const QList<QVariant> &data, TreeItem *parent = 0)
{
	m_parentitem = parent;
	m_itemdata = data;
}

CObservationTreeItem::~CObservationTreeItem()
{
	qDeleteAll(m_childItems);
}

CObservationTreeItem::appendChild(TreeItem *item)
{
	m_childitems.append(item);
}

CObservationTreeItem *CObservationTreeItem::child(int row)
{
	return m_childitems.value(row);
}

int CObservationTreeItem::childCount() const
{
	return m_childitems.count();
}

int CObservationTreeItem::row() const
{
	if(m_parentitem)
		return m_parentitem->m_childitems.indexOf(const_cast<TreeItem*>(this));

	return 0;
}

int CObservationTreeItem::columnCount() const
{
	return m_itemData.count();
}

QVariant data(int column) const
{
	return m_itemData(column);
}

CObservationTreeItem *parentItem()
{
	return m_parentitem;
}
