#include "qnode.h"

float a,b;
int x=0,x1, count=0;
float a_c,b_c,rad;
double ang, x_pose, y_pose , twist;
double roll, pitch, yaw;
geometry_msgs::Quaternion quat;

void poseclearCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    //ROS_INFO_STREAM("Pose message :["<<msg->latitude<<"]["<<msg->longitude<<"]");
    a_c = msg->latitude;
    b_c = msg->longitude;
    rad = msg->position_covariance[0];
    //ROS_INFO_STREAM("Pose_clear message rad: " << rad);
}


void poseCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    x++;
    //ROS_INFO_STREAM("Pose message :["<<msg->latitude<<"]["<<msg->longitude<<"]");
    a = msg->latitude;
    b = msg->longitude;

}

void odoCallback(const nav_msgs::OdometryConstPtr& odo_msg)

{
    quat = odo_msg->pose.pose.orientation;
    x_pose = odo_msg->pose.pose.position.x;
    y_pose = odo_msg->pose.pose.position.y;
    twist= sqrt(pow(odo_msg->twist.twist.linear.x,2) +pow(odo_msg->twist.twist.linear.y,2) + pow(odo_msg->twist.twist.linear.z,2));
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ang=yaw;
    //ROS_INFO_STREAM("Odo message: " << ang);
}


QNode::QNode(int argc, char** argv )
{
    init_argc=argc;
    init_argv=argv;
}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
}

bool QNode::init() {
    std::string top1, top2,top3, pub1;
    if ( ! ros::master::check() ) {
        return false;
    }

    ros::NodeHandle n;
    time = ros::Time::now().toSec(); // начало отсчета
    hear=0;

    n.getParam("/topic_pose",top1);
    n.getParam("/topic_odo",top2);
    n.getParam("/topic_pose_clear",top3);
    n.getParam("/publish_task_point",pub1);
    // Add your ros communications here.
    sub_pose= n.subscribe(top1, 1000, poseCallback);
    sub_odo = n.subscribe(top2, 1000, odoCallback);
    sub_pose_clear = n.subscribe(top3, 1000, poseclearCallback);
    pub_point = n.advertise<geometry_msgs::Pose>(pub1,1000);

    return true;
}

void QNode::update()
{
    if((taskpoint.position.x != taskpoint1.position.x) || (taskpoint.position.y != taskpoint1.position.y))  //Чтобы отправлять только новые данные по топику
    {
        pub_point.publish(taskpoint);
        taskpoint1=taskpoint;
    }
    lat=a;
    ln=b;
    angle=ang;
    xpose=x_pose;
    ypose=y_pose;
    rad_point=rad;
    lat_clear=a_c;
    ln_clear = b_c;
    hear=x;
    time1 = ros::Time::now().toSec() - time; // время от начала измерений
    twist_lin=twist;
    ros::spinOnce();
}
