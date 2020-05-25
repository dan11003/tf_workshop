
#include <ros/ros.h>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <fstream>


class Imu2Tf {

public:

  Imu2Tf(ros::NodeHandle param_nh)
  {

    param_nh.param<std::string>("odom_parent_link", imu_parent_link_, std::string("/world"));
    param_nh.param<std::string>("odom_child_link", imu_child_link_, std::string("/imu_orientation"));
    param_nh.param<std::string>("imu_topic", imu_topic_, std::string("/imu/data"));
    //param_nh.param<std::string>("offset_link", offset_link_, std::string("/velodyne"));
    //param_nh.param<bool>("publish_tf_offset", publish_offset_as_tf, false);
    param_nh.param<bool>("convert_odom_to_tf", publish_imu_as_tf, true);

    double x,y,z,ex,ey,ez;
    param_nh.param<double>("offset_x", x, 0);
    param_nh.param<double>("offset_y", y, 0);
    param_nh.param<double>("offset_z", z, 0);
    param_nh.param<double>("offset_ex", ex, 0);
    param_nh.param<double>("offset_ey", ey, 0);
    param_nh.param<double>("offset_ez", ez, 0);

    offset_tf_.setOrigin(tf::Vector3(x,y,z));
    tf::Quaternion q;
    q.setRPY(ex, ey, ez);
    offset_tf_.setRotation(q);
    std::cout<<"subscribe: "<<imu_topic_<<std::endl;
    imu_sub_ = param_nh.subscribe(imu_topic_, 100, &Imu2Tf::MsgCallback, this);
  }

  void MsgCallback(const sensor_msgs::Imu::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    std::cout<<"callback"<<std::endl;
    tf::Transform odom, offset_transform;
    tf::Quaternion q;
    q.setX(  msg_in->orientation.x);
    q.setY(  msg_in->orientation.y);
    q.setZ(  msg_in->orientation.z);
    q.setW(  msg_in->orientation.w);
    odom.setRotation(q);
    if(publish_imu_as_tf)
      tf_.sendTransform(tf::StampedTransform(odom, msg_in->header.stamp, imu_parent_link_, imu_child_link_));
    //if(publish_offset_as_tf)
    //  tf_.sendTransform(tf::StampedTransform(offset_tf_, msg_in->header.stamp, odom_child_link_, offset_link_));


  }

private:

  std::string imu_topic_;
  std::string imu_parent_link_, imu_child_link_;
  std::string offset_link_;
  bool publish_offset_as_tf, publish_imu_as_tf;
  tf::Transform offset_tf_;
  tf::TransformBroadcaster tf_;
  ros::Subscriber imu_sub_;
  ros::Publisher output_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle param("~");
  Imu2Tf t(param);
  ros::spin();
  return 0;
}



