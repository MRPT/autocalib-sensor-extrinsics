#include "CObservationTreeItem.h"
#include <algorithm>

using namespace mrpt::obs;

CObservationTreeItem::CObservationTreeItem(const std::string &id, const CObservation::Ptr observation, CObservationTreeItem *parent, int prior_index)
{
	m_parentitem = parent;
	m_id = id;
	m_observation = observation;
	m_prior_index = prior_index;
}

CObservationTreeItem::~CObservationTreeItem()
{
	for(auto iter = m_childitems.begin(); iter != m_childitems.end(); iter++)
	{
		delete (*iter);
	}

	m_childitems.clear();
}

void CObservationTreeItem::appendChild(CObservationTreeItem *item)
{
	m_childitems.push_back(item);
}

CObservationTreeItem *CObservationTreeItem::child(int row) const
{
	return m_childitems.at(row);
}

int CObservationTreeItem::childCount() const
{
	return m_childitems.size();
}

int CObservationTreeItem::row() const
{
	if(m_parentitem)
	{
		auto iter = std::find(m_parentitem->m_childitems.begin(), m_parentitem->m_childitems.end(), const_cast<CObservationTreeItem*>(this));
		if(iter != m_parentitem->m_childitems.end())
			return std::distance(m_parentitem->m_childitems.begin(), iter);
	}

	return 0;
}

CObservationTreeItem *CObservationTreeItem::parentItem() const
{
	return this->m_parentitem;
}

std::string CObservationTreeItem::itemId() const
{
	return this->m_id;
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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CObservationTreeItem::cloud()
{
	return this->m_cloud;
}

void CObservationTreeItem::setCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	this->m_cloud = cloud;
}
