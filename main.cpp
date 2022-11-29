#include <CMainWindow.h>
#include <QApplication>
#include <QWidget>

int main(int argc, char *argv[])
{
	QApplication application(argc, argv);

	QApplication::setOrganizationName("MRPT");
	QApplication::setOrganizationDomain("mrpt.org");
	QApplication::setApplicationName("autocalib-sensor-extrinsics");

	CMainWindow main_window;
	main_window.showMaximized();

	return application.exec();
}
