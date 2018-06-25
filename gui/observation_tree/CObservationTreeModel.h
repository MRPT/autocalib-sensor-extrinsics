#pragma once

#ifndef Q_MOC_RUN
#include <observation_tree/CObservationTreeItem.h>
#endif

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

		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		mrpt::obs::CObservation::Ptr observationData(const QModelIndex &index) const;
		QModelIndex index(int row, int column, const QModelIndex &parent) const;
		QModelIndex parent(const QModelIndex &index) const;
		Qt::ItemFlags flags(const QModelIndex &index) const;
		int rowCount(const QModelIndex &parent = QModelIndex()) const;
		int columnCount(const QModelIndex &parent = QModelIndex()) const;
		CObservationTreeItem *getRootItem() const;

		QStringList m_obs_labels;

	private:
		CObservationTreeItem *m_rootitem;
};
