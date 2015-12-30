#include "mainwindow.h"
#include <qt4/QtGui/QtGui>
#include <qt4/QtGui/QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"project_yamap");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    QApplication a(argc, argv);
    MainWindow w(argc,argv);
    w.show();

    return a.exec();
}
