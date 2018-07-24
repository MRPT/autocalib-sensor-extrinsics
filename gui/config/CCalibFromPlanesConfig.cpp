#include <CMainWindow.h>
#include <config/CCalibFromPlanesConfig.h>
#include <ui_CCalibFromPlanesConfig.h>

CCalibFromPlanesConfig::CCalibFromPlanesConfig(mrpt::config::CConfigFile &config_file, QWidget *parent) :
	QWidget(parent),
    m_config_file(config_file),
    m_ui(new Ui::CCalibFromPlanesConfig)
{
	m_ui->setupUi(this);

	std::string ne_method_string = m_config_file.read_string("plane_segmentation", "normal_estimation_method", "COVARIANCE_MATRIX", true);

	if(ne_method_string == "AVERAGE_3D_GRADIENT")
		m_ui->ne_method_cbox->setCurrentIndex(1);
	else if(ne_method_string == "AVERAGE_DEPTH_CHANGE")
		m_ui->ne_method_cbox->setCurrentIndex(2);
	else
		m_ui->ne_method_cbox->setCurrentIndex(0);

	if(m_config_file.read_bool("plane_segmentation", "depth_dependent_smoothing", true, true))
		m_ui->depth_dependent_smoothing_check->setCheckState(Qt::Checked);
	else
		m_ui->depth_dependent_smoothing_check->setCheckState(Qt::Unchecked);

	m_ui->max_depth_change_factor_sbox->setValue(m_config_file.read_double("plane_segmentation", "max_depth_change_factor", 0.02, true));
	m_ui->normal_smoothing_size_sbox->setValue(m_config_file.read_double("plane_segmentation", "normal_smoothing_size", 10.00, true));
	m_ui->angle_threshold_sbox->setValue(m_config_file.read_double("plane_segmentation", "angle_threshold", 4.00, true));
	m_ui->distance_threshold_sbox->setValue(m_config_file.read_double("plane_segmentation", "distance_threshold", 0.05, true));
	m_ui->minimum_threshold_sbox->setValue(m_config_file.read_double("plane_segmentation", "min_inliers_frac", 0.001, true));
	m_ui->max_curvature_sbox->setValue(m_config_file.read_double("plane_segmentation", "max_curvature", 0.1, true));
	m_ui->normals_dot_sbox->setValue(m_config_file.read_double("plane_matching", "normals_dot_product", 0.9, true));
	m_ui->dist_diff_sbox->setValue(m_config_file.read_double("plane_matching", "plane_dist_diff", 0.2, true));

	connect(m_ui->extract_planes_button, SIGNAL(clicked(bool)), this, SLOT(extractPlanes()));
	connect(m_ui->match_planes_button, SIGNAL(clicked(bool)), this, SLOT(matchPlanes()));
	connect(m_ui->calib_button, SIGNAL(clicked(bool)), this, SLOT(calibrate()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalib()));
	connect(m_ui->save_params_button, SIGNAL(clicked(bool)), this, SLOT(saveParamsClicked()));

	m_ui->match_planes_button->setDisabled(true);
	m_ui->calib_button->setDisabled(true);
	m_ui->save_calib_button->setDisabled(true);
}

CCalibFromPlanesConfig::~CCalibFromPlanesConfig()
{
	delete m_ui;
}

void CCalibFromPlanesConfig::extractPlanes()
{
	m_params.seg.normal_estimation_method = m_ui->ne_method_cbox->currentIndex();
	m_params.seg.depth_dependent_smoothing = m_ui->depth_dependent_smoothing_check->isChecked();
	m_params.seg.max_depth_change_factor = m_ui->max_depth_change_factor_sbox->value();
	m_params.seg.normal_smoothing_size = m_ui->normal_smoothing_size_sbox->value();
	m_params.seg.angle_threshold = m_ui->angle_threshold_sbox->value();
	m_params.seg.dist_threshold = m_ui->distance_threshold_sbox->value();
	m_params.seg.min_inliers_frac = m_ui->minimum_threshold_sbox->value();
	m_params.seg.max_curvature = m_ui->max_curvature_sbox->value();
	m_params.calib_status = CalibrationStatus::YET_TO_START;

	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromPlanes(&m_params);
	m_ui->match_planes_button->setDisabled(false);
}

void CCalibFromPlanesConfig::matchPlanes()
{
	m_params.match.normals_dot_prod = m_ui->normals_dot_sbox->value();
	m_params.match.dist_diff = m_ui->dist_diff_sbox->value();

	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromPlanes(&m_params);
}

void CCalibFromPlanesConfig::calibrate()
{
}

void CCalibFromPlanesConfig::saveCalib()
{
}

void CCalibFromPlanesConfig::saveParamsClicked()
{
	m_config_file.write<std::string>("plane_segmentation", "normal_estimation_method", m_ui->ne_method_cbox->currentText().toStdString());

	if(m_ui->depth_dependent_smoothing_check->isChecked())
		m_config_file.write<bool>("plane_segmentation", "depth_dependent_smoothing", true);
	else
		m_config_file.write<bool>("plane_segmentation", "depth_dependent_smoothing", false);

	m_config_file.write<double>("plane_segmentation", "max_depth_change_factor", m_ui->max_depth_change_factor_sbox->value());
	m_config_file.write<double>("plane_segmentation", "normal_smoothing_size", m_ui->normal_smoothing_size_sbox->value());
	m_config_file.write<double>("plane_segmentation", "angle_threshold", m_ui->angle_threshold_sbox->value());
	m_config_file.write<double>("plane_segmentation", "min_inliers_frac", m_ui->minimum_threshold_sbox->value());
	m_config_file.write<double>("plane_segmentation", "distance_threshold", m_ui->distance_threshold_sbox->value());
	m_config_file.write<double>("plane_segmentation", "max_curavture", m_ui->max_curvature_sbox->value());

	// causes segfault
	//m_config_file.writeNow();

	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->saveParams();
}
