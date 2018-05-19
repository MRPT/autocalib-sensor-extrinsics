#pragma once

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include "CObservationTreeItem.h"

//Data model for representing the rawlog observations and interfacing with TreeView

class CObservationTreeModel : public QAbstractItemModel
{
	Q_OBJECT

	public:
		CObservationTreeModel(
				const mrpt::obs::CRawlog &rawlog, QObject *parent = nullptr);
		~CObservationTreeModel();

		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		QModelIndex index(int row, int column, const QModelIndex &parent) const;
		QModelIndex parent(const QModelIndex &index) const;
		int rowCount(const QModelIndex &parent = QModelIndex()) const;
		int columnCount(const QModelIndex &parent = QModelIndex()) const;
	
	private:
		void setupModelData(const QStringList &lines, CTreeItem *parent);

		CObservationTreeItem *m_rootitem;
}
