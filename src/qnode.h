#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <string>
#include <qt4/QtCore/QtCore>
#include <qt4/QtGui/QtGui>
#include <std_msgs/String.h>
#include "sensor_msgs/NavSatFix.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <tf/transform_listener.h>
#include <cmath>


class QNode
{
public:
    QNode(int argc, char** argv );
    ~QNode();
    bool init();
    void update();
    float getlat(){return lat;}
    float getln(){return ln;}
    float getlatclear(){return lat_clear;}
    float getlnclear(){return ln_clear;}
    float getrad(){return rad_point;}
    double getxpose(){return xpose;}
    double getypose(){return ypose;}
    double gettwist_lin(){return twist_lin;}
    double gettime(){return time1;}
    int gethear(){return hear;}
    qreal getang(){return angle;}
    void setlat(float a){lat=a;}
    void setln(float b){ln=b;}
    void setang(qreal ang){angle = ang;}
    void sethear(int c){hear=c;}
    void settaskpoint(float a, float b){taskpoint.position.x = a; taskpoint.position.y = b;}
    void setgpps(int f){gpps=f;}
private:
    int init_argc;
    int gpps; // для попытки сделать постоянную прослушку
    char** init_argv;
    ros::Subscriber sub_pose, sub_odo, sub_pose_clear; //прием топиков
    ros::Publisher pub_point; // отправка топика
    geometry_msgs::Pose taskpoint,taskpoint1; // целевые точки
    int hear; // проверка, идут ли данные
    double xpose,ypose, twist_lin; //Точка odo, линейная скорость
    float lat, ln, lat_clear, ln_clear; // широта и долгота калман и чистые данные
    float rad_point;
    qreal angle;
    double time,time1; // время

};

#endif // QNODE_H
