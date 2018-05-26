#include "CCalibConfig.h"
#include "ui_CCalibConfig.h"

CCalibConfig::CCalibConfig(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::CCalibConfig)
{
	ui->setupUi(this);
}

CCalibConfig::~CCalibConfig()
{
	delete ui;
}
