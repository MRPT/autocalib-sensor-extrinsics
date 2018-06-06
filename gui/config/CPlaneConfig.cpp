#include <config/CPlaneConfig.h>
#include <ui_CPlaneConfig.h>
#include <CMainWindow.h>
#include <QDebug>

CPlaneConfig::CPlaneConfig(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CPlaneConfig)
{
	m_ui->setupUi(this);

	connect(m_ui->run_calib_button, SIGNAL(clicked(bool)), this, SLOT(runCalib()));
	connect(m_ui->proceed_calib_button, SIGNAL(clicked(bool)), this, SLOT(proceedCalib()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalib()));
}

CPlaneConfig::~CPlaneConfig()
{
	delete m_ui;
}

void CPlaneConfig::runCalib()
{

	//TODO
	//obtain segmentation values
	//obtain pointer to const observation model
	//obtain pointer to const viewer ui

	// Get the current state of the initial calibration spin boxes
	CMainWindow *main_window = qobject_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget());
	std::array<double,6> init_calib = main_window->getInitCalib();
}

void CPlaneConfig::proceedCalib()
{

}

void CPlaneConfig::saveCalib()
{

}

//CPlaneMatchingParams CPlaneConfig::getParams()
//{
//	CPlaneMatchingParams params;

//	params.angle_threshold = m_ui->angle_threshold_sbox->value();
//	params.dist_threshold = m_ui->distance_threshold_sbox->value();
//	params.min_inliers_frac = m_ui->minimum_threshold_sbox->value();

//	return params;
//}
