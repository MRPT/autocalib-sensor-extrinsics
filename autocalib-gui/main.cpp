#include "mainwindow.h"
#include <QApplication>
#include <QWidget>

int main(int argc, char *argv[])
{
    QApplication application(argc, argv);
    MainWindow mainWindow;

    mainWindow.show();

    return application.exec();
}
