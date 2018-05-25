#pragma once

#include <QWidget>

namespace Ui {
class CViewerContainer;
}

class CViewerContainer : public QWidget
{
	Q_OBJECT

public:
	explicit CViewerContainer(QWidget *parent = 0);
	~CViewerContainer();

	void changeOutputText(const QString &);

private:
	Ui::CViewerContainer *m_ui;
};
