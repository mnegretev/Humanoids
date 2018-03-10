#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    legLeftIgnoreValueChanged = false;
    legRightIgnoreValueChanged = false;
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
    //Connect signals for the sliders in the joint tab
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
    //Connect signals for controls in the inverse kinematics tab
    QObject::connect(ui->sbLegLeftX,     SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeftY,     SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeftZ,     SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeftRoll,  SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeftPitch, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeftYaw,   SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftCartesianChanged(double)));
    QObject::connect(ui->sbLegLeft0, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegLeft1, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegLeft2, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegLeft3, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegLeft4, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegLeft5, SIGNAL(valueChanged(double)), this, SLOT(txtLegLeftArticularChanged(double)));
    QObject::connect(ui->sbLegRightX,     SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRightY,     SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRightZ,     SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRightRoll,  SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRightPitch, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRightYaw,   SIGNAL(valueChanged(double)), this, SLOT(txtLegRightCartesianChanged(double)));
    QObject::connect(ui->sbLegRight0, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));
    QObject::connect(ui->sbLegRight1, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));
    QObject::connect(ui->sbLegRight2, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));
    QObject::connect(ui->sbLegRight3, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));
    QObject::connect(ui->sbLegRight4, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));
    QObject::connect(ui->sbLegRight5, SIGNAL(valueChanged(double)), this, SLOT(txtLegRightArticularChanged(double)));

    txtLegLeftArticularChanged(0);
    txtLegRightArticularChanged(0);
    
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

//
//SLOTS FOR SIGNAL EMITTED BY THE INVERSE KINEMATICS CONTROLS
//
void MainWindow::txtLegLeftCartesianChanged(double val)
{
    if(legLeftIgnoreValueChanged)
        return;

    legLeftIgnoreValueChanged = true;
    //std::cout << "GUI.->Calculating inverse kinematics for left leg..." << std::endl;
    std::vector<float> joint_values;
    if(!qtRosNode->callIKLegLeft(ui->sbLegLeftX->value(), ui->sbLegLeftY->value(), ui->sbLegLeftZ->value(),
                         ui->sbLegLeftRoll->value(), ui->sbLegLeftPitch->value(), ui->sbLegLeftYaw->value(),
                             joint_values))
    {
        std::cout << "GUI->Cannot calculate inverse kinematics for the left leg :'( " << std::endl;
        legLeftIgnoreValueChanged = false;
        return;
    }
    ui->sbLegLeft0->setValue(joint_values[0]);
    ui->sbLegLeft1->setValue(joint_values[1]);
    ui->sbLegLeft2->setValue(joint_values[2]);
    ui->sbLegLeft3->setValue(joint_values[3]);
    ui->sbLegLeft4->setValue(joint_values[4]);
    ui->sbLegLeft5->setValue(joint_values[5]);

    qtRosNode->publishLegLeftGoalPose(joint_values);
    
    legLeftIgnoreValueChanged = false;
}

void MainWindow::txtLegLeftArticularChanged(double val)
{
    if(legLeftIgnoreValueChanged)
        return;

    legLeftIgnoreValueChanged = true;
    //std::cout << "GUI.->Calculating direct kinematics for left leg..." << std::endl;
    std::vector<float> joint_values;
    float x, y, z, roll, pitch, yaw;
    joint_values.resize(6);
    joint_values[0] = ui->sbLegLeft0->value();
    joint_values[1] = ui->sbLegLeft1->value();
    joint_values[2] = ui->sbLegLeft2->value();
    joint_values[3] = ui->sbLegLeft3->value();
    joint_values[4] = ui->sbLegLeft4->value();
    joint_values[5] = ui->sbLegLeft5->value();
    qtRosNode->callDKLegLeft(joint_values, x, y, z, roll, pitch, yaw);
    ui->sbLegLeftX->setValue(x);    
    ui->sbLegLeftY->setValue(y);
    ui->sbLegLeftZ->setValue(z);
    ui->sbLegLeftRoll->setValue(roll);
    ui->sbLegLeftPitch->setValue(pitch);
    ui->sbLegLeftYaw->setValue(yaw);

    qtRosNode->publishLegLeftGoalPose(joint_values);
    
    legLeftIgnoreValueChanged = false;
}

void MainWindow::txtLegRightCartesianChanged(double val)
{
    if(legRightIgnoreValueChanged)
        return;
    
    legRightIgnoreValueChanged = true;
    //std::cout << "GUI.->Calculating inverse kinematics for right leg..." << std::endl;
    std::vector<float> joint_values;
    if(!qtRosNode->callIKLegRight(ui->sbLegRightX->value(), ui->sbLegRightY->value(), ui->sbLegRightZ->value(),
                         ui->sbLegRightRoll->value(), ui->sbLegRightPitch->value(), ui->sbLegRightYaw->value(),
                             joint_values))
    {
        std::cout << "GUI->Cannot calculate inverse kinematics for the right leg :'( " << std::endl;
        legRightIgnoreValueChanged = false;
        return;
    }
    ui->sbLegRight0->setValue(joint_values[0]);
    ui->sbLegRight1->setValue(joint_values[1]);
    ui->sbLegRight2->setValue(joint_values[2]);
    ui->sbLegRight3->setValue(joint_values[3]);
    ui->sbLegRight4->setValue(joint_values[4]);
    ui->sbLegRight5->setValue(joint_values[5]);

    qtRosNode->publishLegRightGoalPose(joint_values);
    
    legRightIgnoreValueChanged = false;
}

void MainWindow::txtLegRightArticularChanged(double val)
{
    if(legRightIgnoreValueChanged)
        return;

    legRightIgnoreValueChanged = true;
    //std::cout << "GUI.->Calculating direct kinematics for right leg..." << std::endl;
    std::vector<float> joint_values;
    float x, y, z, roll, pitch, yaw;
    joint_values.resize(6);
    joint_values[0] = ui->sbLegRight0->value();
    joint_values[1] = ui->sbLegRight1->value();
    joint_values[2] = ui->sbLegRight2->value();
    joint_values[3] = ui->sbLegRight3->value();
    joint_values[4] = ui->sbLegRight4->value();
    joint_values[5] = ui->sbLegRight5->value();
    qtRosNode->callDKLegRight(joint_values, x, y, z, roll, pitch, yaw);
    ui->sbLegRightX->setValue(x);    
    ui->sbLegRightY->setValue(y);
    ui->sbLegRightZ->setValue(z);
    ui->sbLegRightRoll->setValue(roll);
    ui->sbLegRightPitch->setValue(pitch);
    ui->sbLegRightYaw->setValue(yaw);

    qtRosNode->publishLegRightGoalPose(joint_values);
    
    legRightIgnoreValueChanged = false;
}
