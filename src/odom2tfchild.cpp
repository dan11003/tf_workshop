
#include <ros/ros.h>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <fstream>


class Odom2Tf {

public:

  Odom2Tf(ros::NodeHandle param_nh)
  {

    param_nh.param<std::string>("odom_parent_link", odom_parent_link_, std::string("/world"));
    param_nh.param<std::string>("odom_child_link", odom_child_link_, std::string("/base_link"));
    param_nh.param<std::string>("odom_topic", odom_topic_, std::string("/state_base_link"));
    param_nh.param<std::string>("offset_link", offset_link_, std::string("/velodyne"));
    param_nh.param<bool>("publish_tf_offset", publish_offset_as_tf, false);
    param_nh.param<bool>("convert_odom_to_tf", publish_odom_as_tf, false);

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
    odom_sub_ = param_nh.subscribe(odom_topic_, 100, &Odom2Tf::MsgCallback, this);
  }

  void MsgCallback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    tf::Transform odom, offset_transform;
    tf::poseMsgToTF(msg_in->pose.pose, odom);
    if(publish_odom_as_tf)
      tf_.sendTransform(tf::StampedTransform(odom, msg_in->header.stamp, odom_parent_link_, odom_child_link_));
    if(publish_offset_as_tf)
      tf_.sendTransform(tf::StampedTransform(offset_tf_, msg_in->header.stamp, odom_child_link_, offset_link_));

  }

private:

  std::string odom_topic_;
  std::string odom_parent_link_, odom_child_link_;
  std::string offset_link_;
  bool publish_offset_as_tf, publish_odom_as_tf;
  tf::Transform offset_tf_;
  tf::TransformBroadcaster tf_;
  ros::Subscriber odom_sub_;
  ros::Publisher output_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_map_fuser_node");
  ros::NodeHandle param("~");
  Odom2Tf t(param);
  ros::spin();
  return 0;
}



