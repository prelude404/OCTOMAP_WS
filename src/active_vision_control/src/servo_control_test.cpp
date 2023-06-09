#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
using namespace std;
using namespace Eigen;
double l1=69; //mm
double l2=10.5; //mm
double l3=78; //mm
Matrix4d current_end_pose ;
Matrix4d world_end_pose;
Vector2d current_servo_angle(2,2);//rad
Vector3d current_robot_end_pose(500,1000,1000);
const Vector3d base(1,2,0);//主动视觉基坐标系在世界坐标系下的位置


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
    trans <<1,0,0,base(0),
                    0,1,0,base(1),
                    0,0,1,base(2),
                    0,0,0,1;
    world_end_pose=trans*current_end_pose;
}

void thetaCalculate()
{
    double theta1=atan2(base(0)-current_robot_end_pose(0),current_robot_end_pose(1)-base(1));//rad
    double dis=sqrt(pow(base(0)-l1*sin(theta1)-current_robot_end_pose(0),2)+pow(base(1)-l2*cos(theta1)-current_robot_end_pose(1),2));
    double theta2=3.1415926535/2-atan2(current_robot_end_pose(2)-l1,dis);
    current_servo_angle << theta1,theta2; 
}
int main(int argc, char **argv){
    current_end_pose << 1,0,0,0,
                                                0,1,0,l2,
                                                0,0,1,l1+l3,
                                                0,0,0,1;
    update_end_pose();
    cout<<world_end_pose<<endl;
    thetaCalculate();
    cout<<current_servo_angle<<endl;
    return 0;
}