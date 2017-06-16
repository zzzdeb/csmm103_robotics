#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
//#include <tf/Scalar.h>

using namespace tf;

void tf_publish()
{
    static TransformBroadcaster br;
    Transform obj_tr;
    obj_tr.setIdentity();
    Vector3 t1(0, 1, 1);

    obj_tr.setRotation(tf::Quaternion(0.79, 0.79, 0.0));
    Vector3 obj_origin = obj_tr(t1);
    obj_tr.setOrigin(obj_origin);
    br.sendTransform(tf::StampedTransform(obj_tr, ros::Time::now(), "base_frame", "object_frame"));

    Transform robot_tr;
    robot_tr.setIdentity();
    robot_tr.setRotation(Quaternion(Vector3(0, 0, 1), 1.5));
    Vector3 robot_org = robot_tr(Vector3(0, -1, 0));
    robot_tr.setOrigin(robot_org);
    br.sendTransform(StampedTransform(robot_tr, ros::Time::now(), "base_frame", "robot_frame"));
    Transform camera_tr;
    camera_tr.setIdentity();
    camera_tr.setOrigin(Vector3(0, 0.1, 0.1));
    Vector3 point = camera_tr.inverseTimes(robot_tr.inverse()) * obj_origin;
    Vector3 axis = Vector3(1, 0, 0).cross(point);
    tfScalar dot = Vector3(1, 0, 0).dot(point);
    tfScalar rot_a = tfAcos(dot / (tfScalar(1) * point.length()));
    camera_tr.setRotation(Quaternion(axis, rot_a));
    br.sendTransform(StampedTransform(camera_tr, ros::Time::now(), "robot_frame", "camera_frame"));
}

int main(int c, char **v)
{
    ros::init(c, v, "solution");
    ros::NodeHandle nh;
    ros::Rate r(0.5);
    while (ros::ok())
    {
        tf_publish();
        r.sleep();
    }
}