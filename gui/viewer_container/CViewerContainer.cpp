#include "CViewerContainer.h"
#include "ui_CViewerContainer.h"

CViewerContainer::CViewerContainer(QWidget *parent) :
	QWidget(parent),
	m_ui(new Ui::CViewerContainer)
{
	m_ui->setupUi(this);
}

CViewerContainer::~CViewerContainer()
{
	delete m_ui;
}

void CViewerContainer::changeOutputText(const QString &text)
{
	m_ui->text_output->setText(text);
}
