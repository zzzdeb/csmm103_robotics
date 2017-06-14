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
    
}