#include <CMainWindow.h>
#include <config/CCalibFromLinesConfig.h>
#include <ui_CCalibFromLinesConfig.h>

CCalibFromLinesConfig::CCalibFromLinesConfig(mrpt::config::CConfigFile &config_file, QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CCalibFromLinesConfig),
    m_config_file(config_file)
{
	m_ui->setupUi(this);

	m_ui->clow_threshold_sbox->setValue(m_config_file.read_int("line_segmentation", "canny_low_threshold", 150, true));
	m_ui->chightolow_ratio_sbox->setValue(m_config_file.read_int("line_segmentation", "canny_high_to_low_ratio", 3, true));
	m_ui->ckernel_size_sbox->setValue(m_config_file.read_double("line_segmentation", "canny_kernel_size", 3, true));
	m_ui->hthreshold_sbox->setValue(m_config_file.read_int("line_segmentation", "hough_threshold", 150, true));

	connect(m_ui->extract_lines_button, SIGNAL(clicked(bool)), this, SLOT(extractLines()));
	connect(m_ui->save_calib_button, SIGNAL(clicked(bool)), this, SLOT(saveCalibClicked()));

	m_ui->match_lines_button->setDisabled(true);
	m_ui->calib_button->setDisabled(true);
	m_ui->save_calib_button->setDisabled(true);
}

CCalibFromLinesConfig::~CCalibFromLinesConfig()
{
	delete m_ui;
}

void CCalibFromLinesConfig::extractLines()
{
	m_params.seg.clow_threshold = m_ui->clow_threshold_sbox->value();
	m_params.seg.chigh_to_low_ratio = m_ui->chightolow_ratio_sbox->value();
	m_params.seg.ckernel_size = m_ui->ckernel_size_sbox->value();
	m_params.seg.hthreshold = m_ui->hthreshold_sbox->value();
	m_params.calib_status = CalibrationFromLinesStatus::LCALIB_YET_TO_START;

	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
	m_ui->match_lines_button->setDisabled(false);
}

void CCalibFromLinesConfig::matchLines()
{
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
}

void CCalibFromLinesConfig::calibrate()
{
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines(&m_params);
}

void CCalibFromLinesConfig::saveCalibClicked()
{

}
