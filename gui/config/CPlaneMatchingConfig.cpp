#include <config/CPlaneMatchingConfig.h>
#include <ui_CPlaneMatchingConfig.h>
#include <QDebug>
#include <CMainWindow.h>

CPlaneMatchingConfig::CPlaneMatchingConfig(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CPlaneMatchingConfig)
{
	m_ui->setupUi(this);
	connect(m_ui->run_calib_button, SIGNAL(clicked(bool)), this, SLOT(runCalib()));
	connect(m_ui->proceed_calib_button, SIGNAL(clicked(bool)), this, SLOT(proceedCalib()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalib()));
}

CPlaneMatchingConfig::~CPlaneMatchingConfig()
{
	delete m_ui;
}	

void CPlaneMatchingConfig::runCalib()
{
	TPlaneMatchingParams params;
	params.angle_threshold = m_ui->angle_threshold_sbox->value();
	params.dist_threshold = m_ui->distance_threshold_sbox->value();
	params.min_inliers_frac = m_ui->minimum_threshold_sbox->value();

	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalib(params);
}

void CPlaneMatchingConfig::proceedCalib()
{
}

void CPlaneMatchingConfig::saveCalib()
{
}
