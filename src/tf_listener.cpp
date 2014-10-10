/*

    Daman Bareiss
    DARC Lab @ University of Utah
    
    This node listens to the tf from the mo-cap system and outputs a geometry_msgs::Vector3 of "current_position"

Edit: Matt Beall - Global Yaw and velocity computation

*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/TransformStamped.h"
//#include <tf/transform_datatypes.h>
#include "ros/time.h"
#include <std_msgs/Float32.h>
#include <math.h>
#include <Eigen/Dense>

#define _USE_MATH_DEFINES

using namespace std;

#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3

geometry_msgs::Vector3 pcurr;
geometry_msgs::Vector3 vcurr;
std_msgs::Float32 yaw_glob;
//std_msgs::Float32 roll_glob;
//std_msgs::Float32 pitch_glob;
geometry_msgs::Vector3 rcurr;
geometry_msgs::Vector3 rdotcurr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap");
    ros::NodeHandle node;
       
    ros::Publisher pcurr_pub = node.advertise<geometry_msgs::Vector3>("current_position",1);
    ros::Publisher vcurr_pub = node.advertise<geometry_msgs::Vector3>("current_velocity",1);
    
    ros::Publisher yaw_pub = node.advertise<std_msgs::Float32>("current_yaw",1);
	ros::Publisher r_pub = node.advertise<geometry_msgs::Vector3>("current_r",1);
	ros::Publisher rdot_pub = node.advertise<geometry_msgs::Vector3>("current_rdot",1);
    //ros::Publisher roll_pub = node.advertise<std_msgs::Float32>("current_roll",1);
    //ros::Publisher pitch_pub = node.advertise<std_msgs::Float32>("current_pitch",1);

    tf::TransformListener listener;
    tf::StampedTransform stamped;
    int l_rate = 50;
    ros::Rate loop_rate(l_rate);

    std::vector<double> plast(3);
    Eigen::Matrix3f Rot;
    plast[0]=0;
    plast[1]=0;
    plast[2]=0;
	std::vector<double> rlast(3);
	rlast[0]=0;
	rlast[1]=0;
	rlast[2]=0;

    while(ros::ok())
    {
	listener.waitForTransform("optitrak", "quad", ros::Time(0), ros::Duration(2));
      	listener.lookupTransform("optitrak", "quad",  ros::Time(0), stamped);
        pcurr.x = stamped.getOrigin().getY();
        pcurr.y = -stamped.getOrigin().getX();
        pcurr.z = stamped.getOrigin().getZ();
	
		//Compute Velocities and store position for next step
		//(distance)*(Hz) = distance/second
		vcurr.x = l_rate * (pcurr.x - plast[0]);
		vcurr.y = l_rate * (pcurr.y - plast[1]);
		vcurr.z = l_rate * (pcurr.z - plast[2]);

		plast[0] = pcurr.x;
		plast[1] = pcurr.y;
		plast[2] = pcurr.z; 
		    
		double euler_angles[3];
        pcurr_pub.publish(pcurr);
        vcurr_pub.publish(vcurr);

		/*ROS_INFO("getYaw = %f", tf::getYaw(stamped.getRotation()));*/

		//Get RPY angles, and compute global Yaw
		//btMatrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);        
		tf::Matrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);        
		    
		//Finds Rotation matrix using extrinsic RPY angles then uses inverse kin to find intrinsic. - Matt
		Rot(0,0) = cos(euler_angles[pitch])*cos(euler_angles[yaw]);
		Rot(1,0) = cos(euler_angles[roll])*sin(euler_angles[yaw]) + cos(euler_angles[yaw])*sin(euler_angles[pitch])*sin(euler_angles[roll]);
		Rot(2,0) = sin(euler_angles[yaw])*sin(euler_angles[roll]) - cos(euler_angles[yaw])*cos(euler_angles[roll])*sin(euler_angles[pitch]);
		Rot(0,1) = -cos(euler_angles[pitch])*sin(euler_angles[yaw]);
		Rot(1,1) = cos(euler_angles[yaw])*cos(euler_angles[roll]) - sin(euler_angles[pitch])*sin(euler_angles[yaw])*sin(euler_angles[roll]);
		Rot(2,1) = cos(euler_angles[yaw])*sin(euler_angles[roll]) + cos(euler_angles[roll])*sin(euler_angles[pitch])*sin(euler_angles[yaw]);
		Rot(0,2) = sin(euler_angles[pitch]);
		Rot(1,2) = -cos(euler_angles[pitch])*sin(euler_angles[roll]);
		Rot(2,2) = cos(euler_angles[pitch])*cos(euler_angles[roll]);
	
		rdotcurr.x = l_rate * (rcurr.x - rlast[0]);
		rdotcurr.y = l_rate * (rcurr.y - rlast[1]);
		rdotcurr.z = l_rate * (rcurr.z - rlast[2]);

		rlast[0] = rcurr.x; rlast[1] = rcurr.y; rlast[2] = rcurr.z;

		double theta = acos(((Rot(0,0)+Rot(1,1)+Rot(2,2))-1)*0.5);
		rcurr.x = (0.5*theta/sin(theta))*(Rot(2,1) - Rot(1,2));
		rcurr.y = (0.5*theta/sin(theta))*(Rot(0,2) - Rot(2,0));
		rcurr.z = (0.5*theta/sin(theta))*(Rot(1,0) - Rot(0,1));

		//Intrinsic RPY - Matt
		//roll_glob.data = atan2(Rot(2,1),Rot(2,2));
		//pitch_glob.data = asin(-Rot(2,0));

		yaw_glob.data = atan2(Rot(1,0),Rot(0,0));
		r_pub.publish(rcurr);
		rdot_pub.publish(rdotcurr);
		yaw_pub.publish(yaw_glob);

		//roll_pub.publish(roll_glob);
		//pitch_pub.publish(pitch_glob);

		ros::spinOnce();
		loop_rate.sleep();
    }
}

        
    
