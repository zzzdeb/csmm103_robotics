<<<<<<< HEAD
#include <ros/ros.h>
#include <std_msgs/Int16.h>
//#include <project1_solution/TwoInts.h>


//typedef two_int_talker::TwoInts TwoInts;
ros::Publisher pub;

void callback(TwoInts msg)
{
    TwoInts c = msg.a +msg.b;
    pub.publish(c);
}


int main(int c, char** v)
{
    ros::init(c, v, "solution");
    ros::NodeHandle nh;
    pub = nh.advertise<TwoInts>("sum", 10);
    ros::Subscriber sub = nh.subscribe<TwoInts>('two_ints', callback);
    
=======
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int16.h>
#include "project1_solution/TwoInts.h"

ros::Publisher pub;

void callback(const project1_solution::TwoInts& msg)
{
    std_msgs::Int16 c;
    c.data = msg.a + msg.b;
    pub.publish(c);
}

int main(int c, char** v)
{
    ros::init(c, v, "solution");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Int16>("sum", 10);
    ros::Subscriber sub = nh.subscribe("two_ints", 10, callback);
    ros::spin();
    return 0;
>>>>>>> 14b9b4c3924666c12a1e287398368ceb37278cd6
}