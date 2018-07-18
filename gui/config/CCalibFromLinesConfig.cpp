#include <config/CCalibFromLinesConfig.h>
#include <ui_CCalibFromLinesConfig.h>
#include <CMainWindow.h>

CCalibFromLinesConfig::CCalibFromLinesConfig(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CCalibFromLinesConfig)
{
	m_ui->setupUi(this);
	connect(m_ui->start_calib_button, SIGNAL(clicked(bool)), this, SLOT(startCalib()));
}

CCalibFromLinesConfig::~CCalibFromLinesConfig()
{
	delete m_ui;
}

void CCalibFromLinesConfig::startCalib()
{
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines();
}

void CCalibFromLinesConfig::proceedCalib()
{

}

void CCalibFromLinesConfig::saveCalib()
{

}
