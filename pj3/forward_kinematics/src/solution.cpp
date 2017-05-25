#include <ros/ros.h>
#include <ros/console.h>

// #include <tf/
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transformation.h>
#include <urdf/model.h>
#include <string>
#include <vector>

class ForwardKinematics
{
    public:
    ForwardKinematics(ros::NodeHandle& nh): nh_(nh)
    {
        sub = nh_.subscribe("joint_states", 100, &ForwardKinematics::call_back, this)
        pub = nh_.advertise<>("/tf",,1);
        robot.initParamWithNodeNodeHandle("robot_description");


    }
    ~ForwardKinematics();

    geametry_msgs::TransformStamped convert_to_message(
        , const String& child, const String& parent
    )
    
    void call_back()
    {
        urdf::LinkConstSharedPtr link_name = robot.getRoot();
        vector<String> link_names;
        vector<String> joints;
        while (true)
        {
            if (link_name not in robot.getLinks())
                break;
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Forked kinematic chain!");
                break
            
        }
        all_transfomrs = compute_transforms();
        pub.publish(all_transforms);
    }

    compute_transform()
    {
        
    }

    privat:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Subscriber sub;
    urdf::Model robot;
};

int main(int c, char **v)
{
    ros::init(c, v, "fwk");
    ros::NodeHandle nh;
    ForwardKinematics(nh);
    ros::spin();
    return 0;
}