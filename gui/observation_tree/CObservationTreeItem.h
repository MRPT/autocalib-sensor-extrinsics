#pragma once

#include <QList>
#include <QVariant>

#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

//The type of each item that is stored internally in a tree structure by CObservationTreeModel

class CObservationTreeItem
{
	public:
		CObservationTreeItem(const QVariant &data, const mrpt::obs::CObservation::Ptr observation = 0 /*null when it is just a set (holder) item*/, CObservationTreeItem *parentItem = 0);
		~CObservationTreeItem();

		void appendChild(CObservationTreeItem *child);
		QVariant displayData() const;
		mrpt::obs::CObservation::Ptr getObservation() const;
		CObservationTreeItem *child(int row);
		int childCount() const;
		int row() const;
		CObservationTreeItem *parentItem();

	private:
		QList<CObservationTreeItem*> m_childitems;
		QVariant m_displaydata;
		mrpt::obs::CObservation::Ptr m_observation;
		CObservationTreeItem *m_parentitem;
};
