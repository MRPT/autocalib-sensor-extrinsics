#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/system/datetime.h>

/**
 * Defines the type of each item that
 * is stored internally in CObservationTree.
 */

class CObservationTreeItem
{
	public:

	    /**
		 * Constructor
		 * \param id the item string id.
		 * \param the observation to be contained in the item.
		 * \param parentItem the parent of the item.
		 * \param prior_index the index of the item with respect to the parent in its previous tree, if any.
		 */
	    CObservationTreeItem(const std::string &id, const mrpt::obs::CObservation::Ptr observation = 0, CObservationTreeItem *parentItem = 0,
		                     int prior_index = -1);

		~CObservationTreeItem();

		void appendChild(CObservationTreeItem *child);

		/** Returns the item string id. */
		std::string itemId() const;

		/** Returns a ponter to the contained observation item. */
		mrpt::obs::CObservation::Ptr getObservation() const;

		/** Returns the timestamp of the observation contained in the item. */
		mrpt::system::TTimeStamp getTimeStamp() const;

		/** Returns a pointer to the contained child item, at the specified index, if any. */
		CObservationTreeItem *child(int row) const;

		/** Returns the number of child items the item contains. */
		int childCount() const;

		/** Returns the row count of the item within its parent. */
		int row() const;

		/** Returns the parent item. */
		CObservationTreeItem *parentItem() const;

		/** Returns the index of the item with respect to its prior tree. */
		int getPriorIndex() const;

	private:

		/** List of child items the item contains. */
		std::vector<CObservationTreeItem*> m_childitems;

		/** The item identifier string.
		 * Can be of two types depending on whether the item represents an observation or a set item.
		 * 1) [#item_index] sensor_id : class_name
		 * 2) Observation set #set_index
		 */
		std::string m_id;

		/** Pointer to the observation contained in the item. */
		mrpt::obs::CObservation::Ptr m_observation;

		/** Pointer to the parent of the tree item. */
		CObservationTreeItem *m_parentitem;

		/** The index of the item with respect to the previous tree it was a part of, if any.
		 * For example, the index with respect to the root item in the tree the item belonged to before it was synchronized.
		 */
		int m_prior_index;
};
