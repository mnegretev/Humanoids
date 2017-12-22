#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;
    std::vector<float> legLeftGoalPose;
    std::vector<float> legRightGoalPose;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);

public slots:
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    //Slots for signals emitted by the horizontal sliders
    void sliderLegLeftValueChanged(int val);
    void sliderLegRightValueChanged(int val);

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
