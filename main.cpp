#include <CMainWindow.h>
#include <QApplication>
#include <QWidget>

int main(int argc, char *argv[])
{
	QApplication application(argc, argv);

	QApplication::setOrganizationName("MRPT");
	QApplication::setOrganizationDomain("mrpt.org");
	QApplication::setApplicationName("autocalib-sensor-extrinsics");

	CMainWindow main_window("../../config_files/app_config.ini");
	main_window.showMaximized();

	return application.exec();
}
