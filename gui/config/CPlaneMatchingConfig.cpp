#include <config/CPlaneMatchingConfig.h>
#include <ui_CPlaneMatchingConfig.h>
#include <QDebug>

CPlaneMatchingConfig::CPlaneMatchingConfig(CObservationTreeModel *model, std::array<double,6> *init_calib, std::function<void(const std::string&)> updateFunction, QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CPlaneMatchingConfig)
{
	m_ui->setupUi(this);
	m_model = model;
	m_init_calib = init_calib;

	sendTextUpdate = updateFunction;

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
	CPlaneMatchingParams params;
	params.angle_threshold = m_ui->angle_threshold_sbox->value();
	params.dist_threshold = m_ui->distance_threshold_sbox->value();
	params.min_inliers_frac = m_ui->minimum_threshold_sbox->value();

	CPlaneMatching *plane_matching = new CPlaneMatching(m_model, sendTextUpdate, *m_init_calib, params);
	sendTextUpdate("Running plane-matching algorithm");
	plane_matching->run();
}

void CPlaneMatchingConfig::proceedCalib()
{

}

void CPlaneMatchingConfig::saveCalib()
{

}
