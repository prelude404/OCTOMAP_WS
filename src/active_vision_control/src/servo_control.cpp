#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>
using namespace std;
using namespace Eigen;
double l1=69; //mm
double l2=10.5; //mm
double l3=78; //mm
Matrix4d current_end_pose ;
Matrix4d world_end_pose;
Vector2d current_servo_angle(0,0);//rad
Vector3d current_robot_end_pose(0,0,0);
const Vector3d base(0,-500,-780);//主动视觉基坐标系在世界坐标系下的位置
ros::Publisher servo1_pub;
ros::Publisher servo2_pub;
ros::Publisher end_pose_pub;

void broadcast_tf(){
    //相机根部TF维护
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(base(0)*0.001,base(1)*0.001,0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "/fix_link"));
    //相机末端TF维护
    static tf::TransformBroadcaster br2;
    tf::Transform transform2;
    Vector3d camera_end_pos = world_end_pose.block<3,1>(0,3);
    transform2.setOrigin(tf::Vector3(camera_end_pos(0)*0.001,camera_end_pos(1)*0.001,(camera_end_pos(2)-base(2))*0.001));
    Quaterniond end_quad=Quaterniond(world_end_pose.block<3,3>(0,0));
    tf::Quaternion q2(end_quad.x(),end_quad.y(),end_quad.z(),end_quad.w());
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link", "/camera_link"));
}
void update_end_pose(){
    Matrix4d g0;
    g0<< 1,0,0,0,
                0,1,0,l2,
                0,0,1,l1+l3,
                0,0,0,1;
    Matrix4d e1;
    e1<<cos(current_servo_angle(0)),-sin(current_servo_angle(0)), 0,0,
            sin(current_servo_angle(0)),cos(current_servo_angle(0)),0,0,
            0,0,1,0,
            0,0,0,1;
    Matrix4d e2;
    e2<<1,0,0,0,
            0,cos(current_servo_angle(1)),sin(current_servo_angle(1)),l2*(1-cos(current_servo_angle(1)))-l1*sin(current_servo_angle(1)),
            0,-sin(current_servo_angle(1)),cos(current_servo_angle(1)),l2*sin(current_servo_angle(1))+l2*(1-cos(current_servo_angle(1))),
            0,0,0,1;
    current_end_pose=e1*e2*g0;
    Matrix4d trans ;
    trans << 1,0,0,base(0),
                    0,1,0,base(1),
                    0,0,1,base(2),
                    0,0,0,1;
    world_end_pose=trans*current_end_pose;
    Vector3d test_z=current_robot_end_pose-world_end_pose.block<3,1>(0,3);
    test_z.normalize();
    //cout<<world_end_pose<<endl;
}

void positionCallback(tf::StampedTransform &msg)
{
    current_robot_end_pose << msg.getOrigin().x()*1000, msg.getOrigin().y()*1000, msg.getOrigin().z()*1000;
    double theta1=atan2(base(0)-current_robot_end_pose(0),current_robot_end_pose(1)-base(1));//rad
    double dis=sqrt(pow(base(0)-l1*sin(theta1)-current_robot_end_pose(0),2)+pow(base(1)-l2*cos(theta1)-current_robot_end_pose(1),2));
    double theta2=3.1415926535/2.0-atan2(current_robot_end_pose(2)-l1-base(2),dis);
    current_servo_angle << theta1,theta2; 
    //发布servo消息
    std_msgs::Float64 data1,data2;
    data1.data=current_servo_angle(0)*180.0/3.1415926535;//deg
    data2.data=current_servo_angle(1)*180.0/3.1415926535;//deg
    servo1_pub.publish(data1);
    servo2_pub.publish(data2);
    //cout<<"servo 1 move to"<<data1.data<<endl;
    //cout<<"servo 2 move to"<<data2.data<<endl;
    update_end_pose();
    //发布末端消息
    geometry_msgs::PoseStamped data;
    Vector3d position=world_end_pose.block<3,1>(0,3);
    Quaterniond end_quad=Quaterniond(world_end_pose.block<3,3>(0,0));
    data.pose.position.x=position(0);
    data.pose.position.y=position(1);
    data.pose.position.z=position(2);
    data.pose.orientation.w=end_quad.w();
    data.pose.orientation.x=end_quad.x();
    data.pose.orientation.y=end_quad.y();
    data.pose.orientation.z=end_quad.z();
    end_pose_pub.publish(data);
    broadcast_tf();
}
int main(int argc, char **argv){
    current_end_pose << 1,0,0,0,
                                                0,1,0,l2,
                                                0,0,1,l1+l3,
                                                0,0,0,1;
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;
    servo1_pub = nh.advertise<std_msgs::Float64> ("servo1", 10);  
    servo2_pub = nh.advertise<std_msgs::Float64> ("servo2", 10); 
    end_pose_pub=nh.advertise <geometry_msgs::PoseStamped> ("/camera_pose",10) ;
    std_msgs::Float64 data1,data2;
    data1.data=current_servo_angle(0);
    data2.data=current_servo_angle(1);
    servo1_pub.publish(data1);
    servo2_pub.publish(data2);
    tf::TransformListener listener;  // tf监听器
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            // 查找turtle2与turtle1的坐标变换
            listener.waitForTransform("/world", "/link_6", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/world", "/link_6", ros::Time(0), transform);
            positionCallback(transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
 
        rate.sleep();
    }
    return 0;
}