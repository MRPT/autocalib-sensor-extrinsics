#pragma once

#include <QList>
#include <QVariant>

#include <mrpt/obs/CObservation.h>
#include <mrpt/system/datetime.h>
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

	    /**
		 * Constructor
		 * \param data the string to be displayed in treeview.
		 * \param observation the observation item to be contained, if any.
		 * \param parentItem the parent of the item.
		 * \param prior_index the index of the item with respect to the parent in its previous model, if any.
		 */
	    CObservationTreeItem(const QString &data, const mrpt::obs::CObservation::Ptr observation = 0, CObservationTreeItem *parentItem = 0,
		                     int prior_index = -1);

		~CObservationTreeItem();

		void appendChild(CObservationTreeItem *child);
		QVariant displayData() const;

		/** Returns a ponter to the contained observation item. */
		mrpt::obs::CObservation::Ptr getObservation() const;
		mrpt::system::TTimeStamp getTimeStamp() const;

		/** Returns a pointer to the contained child item, at the specified index, if any. */
		CObservationTreeItem *child(int row) const;

		/** Returns the number of child items the item contains. */
		int childCount() const;

		/** Returns the row count of the item within its parent. */
		int row() const;

		/** Returns the parent item. */
		CObservationTreeItem *parentItem() const;

		/** Returns the index of the item with respect to its prior model. */
		int getPriorIndex() const;

	private:
		/** List of child items the item contains. */
		QList<CObservationTreeItem*> m_childitems;

		/** The string displayed when the model is linked with the treeview. */
		QString m_displaydata;
		mrpt::obs::CObservation::Ptr m_observation;
		CObservationTreeItem *m_parentitem;

		/** The index of the item with respect to the previous model it was a part of, if any.
		 * For example, the index with respect to the parent item in the model before synchronization. Used for cloud visualization purposes.
		 */
		int m_prior_index;
};
