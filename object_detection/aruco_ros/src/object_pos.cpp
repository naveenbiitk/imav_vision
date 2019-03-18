#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265

using namespace std;
using namespace Eigen;

Matrix<float, 3, 3> R;
//Matrix<float, 3, 3> R_inv;
Matrix<float, 3, 3> pos;
geometry_msgs::PoseStamped pos_sp;
Quaternionf quat;

float x = 0, y = 0, z=0;
float x_glob = 0, y_glob = 0, z_glob=0;
int aruco_detected_flag=0;
float sp_thresh=0.1;
float z_dist=0.1;


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion q1(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    //tf::Matrix3x3 m(q1);
    //double r, p,imu_yaw;
    //m.getRPY(r, p, imu_yaw);
    //cout<<r*180/3.14<<"   "<<p*180/3.14<<"   "<<imu_yaw*180/3.14<<endl<<endl;
  
  quat=Eigen::Quaternionf(q1.w(),q1.x(),q1.y(),q1.z());
  R=quat.toRotationMatrix();

}


void odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{
    x_glob = msg->pose.pose.position.x;
    y_glob = msg->pose.pose.position.y;
    z_glob = msg->pose.pose.position.z;
    aruco_detected_flag = 1;
}


void arucocb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    y = -msg->pose.position.x;
    x = -msg->pose.position.y;
    z = -msg->pose.position.z;
    aruco_detected_flag = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_pos");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);
    ros::Subscriber aruco_sub = nh.subscribe("/aruco_single/pose", 10, arucocb);
    ros::Subscriber odom_sub = nh.subscribe("/odometry", 10, odomcb);
   

    ros::Publisher pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/detected_object/pose", 10);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        
        pos_sp.header.stamp = ros::Time::now();
    
        if (aruco_detected_flag == 1 )
        {
            pos(0,0)=x;
            pos(1,0)=y;
            pos(2,0)=z;

           // R_inv = R.inverse();
            pos = R*pos ;

            cout<<x<<"--"<<y<<"****"<<pos(0,0)<<"--"<<pos(1,0)<<endl;
            pos_sp.pose.position.x=pos(0,0)+x_glob;
            pos_sp.pose.position.y=pos(1,0)+y_glob;

           /* if (pos_sp.pose.position.x < -sp_thresh || pos_sp.pose.position.x > sp_thresh || pos_sp.pose.position.y < -sp_thresh || pos_sp.pose.position.y > sp_thresh)

            if (pos_sp.pose.position.x > sp_thresh)
                pos_sp.pose.position.x = sp_thresh;
            else if (pos_sp.pose.position.x < -sp_thresh)
                pos_sp.pose.position.x = -sp_thresh;
            if (pos_sp.pose.position.y < -sp_thresh)
                pos_sp.pose.position.y = -sp_thresh;
            else if (pos_sp.pose.position.y > sp_thresh)
                pos_sp.pose.position.y = sp_thresh;
*/
            
            pos_sp.pose.position.z = z_dist;
            pos_sp_pub.publish(pos_sp);
            
            aruco_detected_flag = 0;
        }

        pos_sp_pub.publish(pos_sp);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}







