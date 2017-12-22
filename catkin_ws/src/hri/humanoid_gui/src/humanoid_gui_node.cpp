#include <iostream>
#include <QApplication>
#include "MainWindow.h"
#include "QtRosNode.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HUMANOID GUI BY MARCOSOFT" << std::endl;
    ros::init(argc, argv, "humanoid_gui");
    ros::NodeHandle n;

    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);
    
    mainWindow.show();
    return app.exec();
}
