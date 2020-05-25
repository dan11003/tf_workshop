/*! By Daniel Adolfsson:
 *
 *  This node simply creates adds a tf transform between "parent_link" and "child_link".
 *  Transformation time is specified by the inoming message. Frequency and timestamp is therefore pererved.
 *
 */


#include <ros/ros.h>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include <time.h>
#include <fstream>
#include "sensor_msgs/Imu.h"
#include "tf_conversions/tf_kdl.h"
#include "tf_conversions/tf_eigen.h"



using std::endl;
using std::cout;
class TFPublisher {

public:

  TFPublisher(ros::NodeHandle param_nh)
  {

    param_nh.param<std::string>("parent_link", parent_link_, std::string("robot3/base_footprint"));
    param_nh.param<std::string>("child_link", child_link_, std::string("/imu"));
    param_nh.param<std::string>("topic", topic_, std::string("/robot3/sensors/imu/data"));
    param_nh.param<std::string>("type", type_, std::string("sensor_msgs::Imu"));

    double x,y,z,ex,ey,ez;
    param_nh.param<double>("sensor_pose_x", x, 0);
    param_nh.param<double>("sensor_pose_y", y, 0);
    param_nh.param<double>("sensor_pose_z", z, 0);
    param_nh.param<double>("sensor_pose_ex", ex, 0);
    param_nh.param<double>("sensor_pose_ey", ey, 0);
    param_nh.param<double>("sensor_pose_ez", ez, 0);


    offset_tf_.setOrigin(tf::Vector3(x,y,z));
    tf::Quaternion q;
    q.setRPY(ex, ey, ez);
    offset_tf_.setRotation(q);
    if(type_=="nav_msgs::Odometry"){
      std::cout<<"TF publisher Odometry, topic="<<topic_<<std::endl;
      sub_ = param_nh.subscribe(topic_, 100, &TFPublisher::OdomMsgCallback, this);
    }
    else if(type_=="sensor_msgs::Imu"){
      sub_ = param_nh.subscribe(topic_, 100, &TFPublisher::ImuMsgCallback, this);
      std::cout<<"TF publisher IMU, topic: "<<topic_<<std::endl;
    }
    else{
      std::cerr<<"TF publisher: Type specified"<<std::endl;
    }
  }

  void OdomMsgCallback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    tf_.sendTransform(tf::StampedTransform(offset_tf_, msg_in->header.stamp, parent_link_, child_link_));
  }
  void ImuMsgCallback(const sensor_msgs::Imu::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    tf_.sendTransform(tf::StampedTransform(offset_tf_, msg_in->header.stamp, parent_link_, child_link_));
    tf::Transform Timu;
    Timu.setIdentity();
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg_in->orientation, q);
    Timu.setRotation(q);
    tf_.sendTransform(tf::StampedTransform(Timu, msg_in->header.stamp, "world", "imu_orientation"));
  }


private:

  std::string topic_;
  std::string parent_link_, child_link_;
  std::string type_;
  tf::Transform offset_tf_;
  tf::TransformBroadcaster tf_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_dynamic_publisher");
  ros::NodeHandle param("~");
  TFPublisher t(param);
  ros::spin();
  return 0;
}



