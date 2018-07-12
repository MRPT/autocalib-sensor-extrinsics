#include <config/CLineMatchingConfig.h>
#include <ui_CLineMatchingConfig.h>
#include <CMainWindow.h>

CLineMatchingConfig::CLineMatchingConfig(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CLineMatchingConfig)
{
	m_ui->setupUi(this);
	connect(m_ui->start_calib_button, SIGNAL(clicked(bool)), this, SLOT(startCalib()));
}

CLineMatchingConfig::~CLineMatchingConfig()
{
	delete m_ui;
}

void CLineMatchingConfig::startCalib()
{
	static_cast<CMainWindow*>(parentWidget()->parentWidget()->parentWidget())->runCalibFromLines();
}

void CLineMatchingConfig::proceedCalib()
{

}

void CLineMatchingConfig::saveCalib()
{

}
