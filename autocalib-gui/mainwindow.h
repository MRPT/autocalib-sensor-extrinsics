#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class QGroupBox;
class QComboBox;
class QWidget;
class QPushButton;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    void createSetup();
    void createInitialization();
    void createInput();
    void createRun();
    void createResults();

    QWidget *centralWidget;

    QGroupBox *setupGroupBox;
    QComboBox *sensorsComboBox;
    QComboBox *algorithmsComboBox;

    QGroupBox *initializationGroupBox;
    QPushButton *continueButton;

    QGroupBox *inputGroupBox;
    QGroupBox *runGroupBox;
    QGroupBox *resultsGroupBox;

private slots:
    void sensorIndexChanged(int);
    void algorithmIndexChanged(int);

};

#endif // MAINWINDOW_H
