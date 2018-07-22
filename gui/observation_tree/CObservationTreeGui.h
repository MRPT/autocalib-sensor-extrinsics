#pragma once

#include <CObservationTree.h>
#include <interfaces/CTextObserver.h>

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

/**
 * Data model for representing the rawlog observations and interfacing them with QTreeView.
 * Inherits from CObservationTree
 */

class CObservationTreeGui : public QAbstractItemModel, public CObservationTree
{
	Q_OBJECT

	public:

	    CObservationTreeGui(const std::string &rawlog_filename, const mrpt::config::CConfigFile &config_file, QObject *parent = 0);
	    ~CObservationTreeGui();

		/** Add an observer (listener) to the list of observers that are to be notified with text updates.
		 * \param observer the listener to be added.
		*/
		void addTextObserver(CTextObserver *observer);

		/**
		 * Notifies the list of observers with a text message.
		 * \param msg the message to be sent.
		 */
		void publishText(const std::string &msg);

		QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
		QModelIndex index(int row, int column, const QModelIndex &parent) const;
		QModelIndex parent(const QModelIndex &index) const;
		Qt::ItemFlags flags(const QModelIndex &index) const;
		int rowCount(const QModelIndex &parent = QModelIndex()) const;
		int columnCount(const QModelIndex &parent = QModelIndex()) const;

		/** Returns the observation item at the specified index in the model.
		 * \param index the model index.
		 */
		CObservationTreeItem* getItem(const QModelIndex &index) const;

	private:

		/** The list of text observers. */
		std::vector<CTextObserver*> m_text_observers;
};
