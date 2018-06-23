#include <config/CPlaneMatchingConfig.h>
#include <ui_CPlaneMatchingConfig.h>
#include <QDebug>
#include <CMainWindow.h>

CPlaneMatchingConfig::CPlaneMatchingConfig(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CPlaneMatchingConfig)
{
	m_ui->setupUi(this);
	connect(m_ui->start_calib_button, SIGNAL(clicked(bool)), this, SLOT(startCalib()));
	connect(m_ui->proceed_calib_button, SIGNAL(clicked(bool)), this, SLOT(proceedCalib()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalib()));

	m_ui->proceed_calib_button->setDisabled(true);
	m_ui->save_calib_button->setDisabled(true);
}

CPlaneMatchingConfig::~CPlaneMatchingConfig()
{
	delete m_ui;
}	

void CPlaneMatchingConfig::startCalib()
{
	TPlaneMatchingParams params;
	params.normal_estimation_method = m_ui->ne_method_cbox->currentIndex();
	params.depth_dependent_smoothing = m_ui->depth_dependent_smoothing_check->isChecked();
	params.max_depth_change_factor = m_ui->max_depth_change_factor_sbox->value();
	params.normal_smoothing_size = m_ui->normal_smoothing_size_sbox->value();
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
