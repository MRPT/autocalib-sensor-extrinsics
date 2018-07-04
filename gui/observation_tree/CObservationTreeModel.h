#pragma once

#include <observation_tree/CObservationTreeItem.h>
#include <utils/CTextObserver.h>

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

//Data model for representing the rawlog observations and interfacing them with TreeView

class CObservationTreeModel : public QAbstractItemModel
{
	Q_OBJECT

	public:
		explicit CObservationTreeModel(const std::string &rawlog_filename, QObject *parent = 0);
		~CObservationTreeModel();
	    void loadModel();
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
		QStringList getObsLabels() const;

	private:
		CObservationTreeItem *m_rootitem;
		QStringList m_obs_labels;
		//store count of observations of each observation type
		std::vector<int> m_count_in_label;
		std::string m_rawlog_filename;
		std::vector<CTextObserver*> m_text_observers;
};
