#include <observation_tree/CObservationTreeItem.h>

#include <QString>

using namespace mrpt::obs;

CObservationTreeItem::CObservationTreeItem(const QString &data, const CObservation::Ptr observation, CObservationTreeItem *parent, int prior_index)
{
	m_parentitem = parent;
	m_displaydata = data;
	m_observation = observation;
	m_prior_index = prior_index;
}

CObservationTreeItem::~CObservationTreeItem()
{
	qDeleteAll(m_childitems);
}

void CObservationTreeItem::appendChild(CObservationTreeItem *item)
{
	m_childitems.append(item);
}

CObservationTreeItem *CObservationTreeItem::child(int row) const
{
	return m_childitems.value(row);
}

int CObservationTreeItem::childCount() const
{
	return m_childitems.count();
}

int CObservationTreeItem::row() const
{
	if(m_parentitem)
		return m_parentitem->m_childitems.indexOf(const_cast<CObservationTreeItem*>(this));

	return 0;
}

CObservationTreeItem *CObservationTreeItem::parentItem() const
{
	return this->m_parentitem;
}

QVariant CObservationTreeItem::displayData() const
{
	return this->m_displaydata;
}

mrpt::system::TTimeStamp CObservationTreeItem::getTimeStamp() const
{
	return this->m_observation->timestamp;
}

CObservation::Ptr CObservationTreeItem::getObservation() const
{
	return this->m_observation;
}

int CObservationTreeItem::getPriorIndex() const
{
	return this->m_prior_index;
}
