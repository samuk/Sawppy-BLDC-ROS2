#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

 # define M_PI           3.14159265358979323846  /* pi */

class ImuQuatToDeg
{
public:
   ImuQuatToDeg(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
   : _nh(node_handle), _pnh(private_node_handle)
   {
       this->init();
   }
   ~ImuQuatToDeg() = default;

   void init();
   void cb_imu_vru2enu(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

   geometry_msgs::Vector3Stamped _orient_rpy;
   // public and private ros node handle
   ros::NodeHandle _nh;
   ros::NodeHandle _pnh;
   // Subscribers and publishers
   ros::Subscriber _sub_imu_vru;
   ros::Publisher  _pub_imu_enu;
};

void ImuQuatToDeg::init()
{
   _sub_imu_vru = _nh.subscribe("/imu_quat", 1000, &ImuQuatToDeg::cb_imu_vru2enu, this);
   _pub_imu_enu = _nh.advertise<geometry_msgs::Vector3Stamped>("/imu_rpy", 1000);
}


void ImuQuatToDeg::cb_imu_vru2enu(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
   // Get message
   _orient_rpy.header = msg->header;

   tf::Quaternion quat;
   double roll, pitch, yaw;
   tf::quaternionMsgToTF(msg->quaternion, quat);
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
   _orient_rpy.vector.x = roll * 180/M_PI;
   _orient_rpy.vector.y = pitch * 180/M_PI;
   _orient_rpy.vector.z = yaw * 180/M_PI;

   // Publish corrected imu message
   _pub_imu_enu.publish(_orient_rpy);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "imu_quat_to_deg");
   ros::NodeHandle nh;
   ros::NodeHandle nh_private("~");
   ImuQuatToDeg node(nh, nh_private);
   ros::spin();

   return 0;
}
