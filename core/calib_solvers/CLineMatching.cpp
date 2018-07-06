#include <calib_solvers/CLineMatching.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace mrpt::obs;

CLineMatching::CLineMatching(CObservationTreeModel *model) :
    m_model(model)
	
{
}

CLineMatching::~CLineMatching()
{
}

void CLineMatching::extractLines()
{
	CObservationTreeItem *root_item, *tree_item;
	CObservation3DRangeScan::Ptr obs_item;

	//testing with the first item alone
	root_item = m_model->getRootItem();
	obs_item = std::dynamic_pointer_cast<CObservation3DRangeScan>(root_item->child(0)->child(0)->getObservation());

	cv::Mat image = cv::cvarrToMat(obs_item->intensityImage.getAs<IplImage>());
}

void CLineMatching::runSegmentation()
{
	double rho, theta;
	double cos_theta, sin_theta;
	double m, c, c_max;
}
