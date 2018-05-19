#pragma once

#include <QList>
#include <QVariant>

//The type of each item that is stored internally in a tree structure by CObservationTreeModel

class CObservationTreeItem
{
	public:
		CObservationTreeItem(const QList<QVariant> &data, CObservationTreeItem *parentItem = 0);
		~CObservationTreeItem();

		void appendChild(CObservationTreeItem *child);

		CObservationTreeItem *child(int row);
		int childCount() const;
		int columnCount() const;
		QVariant data(int column) const;
		int row() const;
		CObservationTreeItem *parentItem();

	private:
		QList<CObservationTreeItem*> m_childitems;
		QList<QVariant> m_itemdata;
		CObservationTreeItem *m_parentitem;
};
