#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->hsLegLeftHipYaw->setValue(50);
    ui->hsLegLeftHipPitch->setValue(50);  
    ui->hsLegLeftHipRoll->setValue(50);
    ui->hsLegLeftKnee->setValue(50); 
    ui->hsLegLeftAnklePitch->setValue(50);
    ui->hsLegLeftAnkleRoll->setValue(50);
    ui->hsLegRightHipYaw->setValue(50);
    ui->hsLegRightHipPitch->setValue(50);
    ui->hsLegRightHipRoll->setValue(50);
    ui->hsLegRightKnee->setValue(50);
    ui->hsLegRightAnklePitch->setValue(50);
    ui->hsLegRightAnkleRoll->setValue(50);
	
    legLeftGoalPose.resize(6);
    legRightGoalPose.resize(6);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;

    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
    QObject::connect(ui->hsLegLeftHipYaw,     SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegLeftHipPitch,   SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegLeftHipRoll,    SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegLeftKnee,       SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegLeftAnklePitch, SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegLeftAnkleRoll,  SIGNAL(valueChanged(int)), this, SLOT(sliderLegLeftValueChanged(int)));
    QObject::connect(ui->hsLegRightHipYaw,     SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
    QObject::connect(ui->hsLegRightHipPitch,   SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
    QObject::connect(ui->hsLegRightHipRoll,    SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
    QObject::connect(ui->hsLegRightKnee,       SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
    QObject::connect(ui->hsLegRightAnklePitch, SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
    QObject::connect(ui->hsLegRightAnkleRoll,  SIGNAL(valueChanged(int)), this, SLOT(sliderLegRightValueChanged(int)));
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    
}


//
//SLOTS FOR SIGNAL EMITTED BY THE SLIDERS
//
void MainWindow::sliderLegLeftValueChanged(int val)
{
    //std::cout << "Slider left value changed" << std::endl;
    legLeftGoalPose[0] = (ui->hsLegLeftHipYaw->value() - 50) / 100.0 * 2 * M_PI;
    legLeftGoalPose[1] = (ui->hsLegLeftHipPitch->value() - 50) / 100.0 * 2 * M_PI;
    legLeftGoalPose[2] = (ui->hsLegLeftHipRoll->value() - 50) / 100.0 * 2 * M_PI;
    legLeftGoalPose[3] = (ui->hsLegLeftKnee->value() - 50) / 100.0 * 2 * M_PI;
    legLeftGoalPose[4] = (ui->hsLegLeftAnklePitch->value() - 50) / 100.0 * 2 * M_PI;
    legLeftGoalPose[5] = (ui->hsLegLeftAnkleRoll->value() - 50) / 100.0 * 2 * M_PI;
    qtRosNode->publishLegLeftGoalPose(legLeftGoalPose);
}

void MainWindow::sliderLegRightValueChanged(int val)
{
    //std::cout << "Slider right value changed" << std::endl;
    legRightGoalPose[0] = (ui->hsLegRightHipYaw->value() - 50) / 100.0 * 2 * M_PI;   
    legRightGoalPose[1] = (ui->hsLegRightHipPitch->value() - 50) / 100.0 * 2 * M_PI; 
    legRightGoalPose[2] = (ui->hsLegRightHipRoll->value() - 50) / 100.0 * 2 * M_PI;  
    legRightGoalPose[3] = (ui->hsLegRightKnee->value() - 50) / 100.0 * 2 * M_PI;     
    legRightGoalPose[4] = (ui->hsLegRightAnklePitch->value() - 50) / 100.0 * 2 * M_PI;
    legRightGoalPose[5] = (ui->hsLegRightAnkleRoll->value() - 50) / 100.0 * 2 * M_PI;
    qtRosNode->publishLegRightGoalPose(legRightGoalPose);
}
