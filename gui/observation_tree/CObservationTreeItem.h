#pragma once

#include <QList>
#include <QVariant>

#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

/**
 * Defines the type of each item that
 * is stored internally in CObservationTreeModel.
 *
 * Detailed description.
 */

class CObservationTreeItem
{
	public:
		CObservationTreeItem(const QVariant &data, const mrpt::obs::CObservation::Ptr observation = 0 /*null when it is just a set (holder) item*/, CObservationTreeItem *parentItem = 0);
		~CObservationTreeItem();

		void appendChild(CObservationTreeItem *child);
		QVariant displayData() const;
		mrpt::obs::CObservation::Ptr getObservation() const;
		CObservationTreeItem *child(int row);
		/** Returns the number of child items it contains */
		int childCount() const;
		/** Returns the row count of the item within its parent. */
		int row() const;
		/** Returns the parent item */
		CObservationTreeItem *parentItem();

	private:
		QList<CObservationTreeItem*> m_childitems;
		QVariant m_displaydata;
		mrpt::obs::CObservation::Ptr m_observation;
		CObservationTreeItem *m_parentitem;
};
