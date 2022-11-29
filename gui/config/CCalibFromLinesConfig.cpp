#include <CMainWindow.h>
#include <config/CCalibFromLinesConfig.h>
#include <ui_CCalibFromLinesConfig.h>

CCalibFromLinesConfig::CCalibFromLinesConfig(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CCalibFromLinesConfig)
{
	m_ui->setupUi(this);

	connect(m_ui->extract_lines_button, SIGNAL(clicked(bool)), this, SLOT(extractLinesClicked()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalibClicked()));
	connect(m_ui->match_lines_button, SIGNAL(clicked(bool)), this, SLOT(matchLinesClicked()));
	connect(m_ui->calib_button, SIGNAL(clicked(bool)), this, SLOT(calibrateClicked()));

	m_ui->match_lines_button->setDisabled(true);
	m_ui->calib_button->setDisabled(true);
	m_ui->save_calib_button->setDisabled(true);
}

CCalibFromLinesConfig::~CCalibFromLinesConfig()
{
	delete m_ui;
}

void CCalibFromLinesConfig::setConfig(mrpt::config::CConfigFile &config_file)
{
	m_config_file = config_file;
	m_ui->downsample_factor_sbox->setValue(m_config_file.read_int("grouped_observations", "downsample_factor", 10, true));
	m_ui->clow_threshold_sbox->setValue(m_config_file.read_int("line_segmentation", "canny_low_threshold", 150, true));
	m_ui->chightolow_ratio_sbox->setValue(m_config_file.read_int("line_segmentation", "canny_high_to_low_ratio", 3, true));
	m_ui->ckernel_size_sbox->setValue(m_config_file.read_double("line_segmentation", "canny_kernel_size", 3, true));
	m_ui->hthreshold_sbox->setValue(m_config_file.read_int("line_segmentation", "hough_threshold", 150, true));
}

void CCalibFromLinesConfig::extractLinesClicked()
{
	m_params.downsample_factor = m_ui->downsample_factor_sbox->value();
	m_params.seg.clow_threshold = m_ui->clow_threshold_sbox->value();
	m_params.seg.chigh_to_low_ratio = m_ui->chightolow_ratio_sbox->value();
	m_params.seg.ckernel_size = m_ui->ckernel_size_sbox->value();
	m_params.seg.hthreshold = m_ui->hthreshold_sbox->value();
	m_params.calib_status = CalibFromLinesStatus::LCALIB_YET_TO_START;
	m_ui->match_lines_button->setDisabled(false);
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
}

void CCalibFromLinesConfig::matchLinesClicked()
{
	m_params.calib_status = CalibFromLinesStatus::LINES_EXTRACTED;
	m_ui->calib_button->setDisabled(false);
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
}

void CCalibFromLinesConfig::calibrateClicked()
{
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
}

void CCalibFromLinesConfig::saveCalibClicked()
{

}
