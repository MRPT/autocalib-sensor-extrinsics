#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGridLayout>
#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    centralWidget = new QWidget(this);
    this->setCentralWidget(centralWidget);

    createSetup();
    createInitialization();
    createInput();
    createRun();
    createResults();

    QGridLayout *mainLayout = new QGridLayout(centralWidget);

    mainLayout->addWidget(setupGroupBox,0,0,1,1);
    mainLayout->addWidget(initializationGroupBox,1,0,1,1);
    mainLayout->addWidget(inputGroupBox,0,1,2,1);
    mainLayout->addWidget(runGroupBox,0,2,2,1);
    mainLayout->addWidget(resultsGroupBox,0,3,2,1);

    mainLayout->setColumnStretch(0,25);
    mainLayout->setColumnStretch(1,25);
    mainLayout->setColumnStretch(2,25);
    mainLayout->setColumnStretch(3,25);

    setWindowTitle("Automatic Sensor Extrinsic Calibration");
}

void MainWindow::createSetup()
{
    setupGroupBox = new QGroupBox(tr("Setup"));
    QFormLayout *layout = new QFormLayout;

    sensorsComboBox = new QComboBox;
    algorithmsComboBox = new QComboBox;
    algorithmsComboBox->setDisabled(true);

    sensorsComboBox->addItem("Choose");
    sensorsComboBox->addItem("3D LiDAR - 3D LiDAR");
    sensorsComboBox->addItem("3D LiDAR - RGB");
    sensorsComboBox->addItem("3D LiDAR - RGB-D");
    sensorsComboBox->addItem("RGB - RGB");
    sensorsComboBox->addItem("RGB - RGB-D");
    sensorsComboBox->addItem("RGB-D - RGB-D");

    connect(sensorsComboBox,SIGNAL(currentIndexChanged(int)),
            this,SLOT(sensorIndexChanged(int)));

    algorithmsComboBox->addItem("Choose");
    algorithmsComboBox->addItem("Line Matching");
    algorithmsComboBox->addItem("Plane matching");
    algorithmsComboBox->addItem("Trajectory matching");

    connect(algorithmsComboBox, SIGNAL(currentIndexChanged(int)),
            this,SLOT(algorithmIndexChanged(int)));

    layout->addRow(new QLabel(tr("Sensor combination")),sensorsComboBox);
    layout->addRow(new QLabel(tr("Calibration algorithm")),algorithmsComboBox);

    setupGroupBox->setLayout(layout);
}

void MainWindow::createInitialization()
{
    initializationGroupBox = new QGroupBox(tr("Initialization"));
    continueButton = new QPushButton(tr("Continue"));
    QVBoxLayout *groupLayout = new QVBoxLayout;
    QFormLayout *layout = new QFormLayout;

    layout->addRow(new QLabel(tr("Tx")),new QLineEdit);
    layout->addRow(new QLabel(tr("Ty")),new QLineEdit);
    layout->addRow(new QLabel(tr("Tz")),new QLineEdit);
    layout->addRow(new QLabel(tr("Rx")),new QLineEdit);
    layout->addRow(new QLabel(tr("Ry")),new QLineEdit);
    layout->addRow(new QLabel(tr("Rz")),new QLineEdit);
    layout->addRow(new QLabel(tr("MED")),new QLineEdit);

    groupLayout->addLayout(layout);
    groupLayout->addWidget(continueButton);

    initializationGroupBox->setLayout(groupLayout);
    initializationGroupBox->setDisabled(true);
}

void MainWindow::createInput()
{
    inputGroupBox = new QGroupBox("Input");
}

void MainWindow::createRun()
{
    runGroupBox = new QGroupBox("Run");
}

void MainWindow::createResults()
{
    resultsGroupBox = new QGroupBox("Results");
}

void MainWindow::sensorIndexChanged(int index)
{
    algorithmsComboBox->setDisabled(false);
}

void MainWindow::algorithmIndexChanged(int index)
{
    initializationGroupBox->setDisabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}
