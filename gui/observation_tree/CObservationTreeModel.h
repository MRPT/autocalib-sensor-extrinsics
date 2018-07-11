#pragma once

#include <observation_tree/CObservationTreeItem.h>
#include <interfaces/CTextObserver.h>

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

/**
 * Data model for representing the rawlog observations and interfacing them with TreeView.
 *
 * Detailed description.
 */

class CObservationTreeModel : public QAbstractItemModel
{
	Q_OBJECT

	public:
	    explicit CObservationTreeModel(QObject *parent = 0);
		~CObservationTreeModel();
	    void loadModel(const std::string &rawlog_filename);
		void addTextObserver(CTextObserver *observer);
		void publishText(const std::string &msg);

		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		mrpt::obs::CObservation::Ptr observationData(const QModelIndex &index) const;
		QModelIndex index(int row, int column, const QModelIndex &parent) const;
		QModelIndex parent(const QModelIndex &index) const;
		Qt::ItemFlags flags(const QModelIndex &index) const;
		int rowCount(const QModelIndex &parent = QModelIndex()) const;
		int columnCount(const QModelIndex &parent = QModelIndex()) const;
		CObservationTreeItem *getRootItem() const;
		void setRootItem(CObservationTreeItem *new_rootitem);
		/** Returns a list of the unique sensor labels found in the rawlog. */
		QStringList getSensorLabels() const;

		/** Groups observations together based on their time stamp proximity.
		 * Stores the results back in the same model.
		 * \param max_delay Maximum allowable delay between observations
		 */
		void syncObservations(const QStringList &selected_sensor_labels, const int &max_delay = 1000);

	private:
		/** The root parent item from which all the other observations originate. */
		CObservationTreeItem *m_rootitem;
		/** The unique sensor labels found in the rawlog. */
		QStringList m_sensor_labels;
		/** store count of observations of each observation type. */
		std::vector<int> m_count_of_label;
		/** The name of the file the rawlog was loaded from. */
		std::string m_rawlog_filename;
		std::vector<CTextObserver*> m_text_observers;
};
