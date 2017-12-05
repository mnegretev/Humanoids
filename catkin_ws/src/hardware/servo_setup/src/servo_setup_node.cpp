#include <iostream>
#include <QApplication>
#include "MainWindow.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SERVO-SETUP NODE..." << std::endl;

    QApplication app(argc, argv);
    MainWindow mainWindow;

    mainWindow.show();
    return app.exec();
}
