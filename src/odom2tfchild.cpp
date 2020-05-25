
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

        param_nh.param<std::string>("odom_topic", odom_topic_, std::string("/state_base_link"));

        param_nh.param<std::string>("odom_parent_link", odom_parent_link_, std::string("/world"));
        param_nh.param<std::string>("odom_child_link", odom_child_link_, std::string("/base_link"));
        param_nh.param<bool>("convert_odom_to_tf", convert_odom_to_tf_, true);
        param_nh.param<bool>("parent_new_frame", use_odom_frame_offset_, false); //publish parent in new fixed frame, initialized with zero
        param_nh.param<bool>("publish_new_frame_odom", publish_new_frame_odom_, false);
        param_nh.param<std::string>("new_frame_odom_topic", new_frame_odom_topic_, std::string("/base_link_new"));


        param_nh.param<std::string>("offset_link", child_link_, std::string("/velodyne"));
        param_nh.param<bool>("publish_additional_child_link", publish_child_link, false);



        double x,y,z,ex,ey,ez;
        param_nh.param<double>("offset_x", x, 0);
        param_nh.param<double>("offset_y", y, 0);
        param_nh.param<double>("offset_z", z, 0);
        param_nh.param<double>("offset_ex", ex, 0);
        param_nh.param<double>("offset_ey", ey, 0);
        param_nh.param<double>("offset_ez", ez, 0);

        if(publish_new_frame_odom_){
            output_pub_ = param_nh.advertise<nav_msgs::Odometry>(new_frame_odom_topic_,100);
            new_frame_odom_msg_.header.frame_id = odom_parent_link_;
        }
        offset_tf_.setOrigin(tf::Vector3(x,y,z));
        tf::Quaternion q;
        q.setRPY(ex, ey, ez);
        offset_tf_.setRotation(q);

        odom_sub_ = param_nh.subscribe(odom_topic_, 100, &Odom2Tf::MsgCallback, this);
    }

    void MsgCallback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
    {
        tf::Transform odom, corrected;
        tf::poseMsgToTF(msg_in->pose.pose, odom);
        if(convert_odom_to_tf_){
            if(use_odom_frame_offset_){
                if(!initialized_){
                    new_odom_frame_ = odom;
                    initialized_ = true;
                }
                corrected = new_odom_frame_.inverse()*odom;
                if(publish_new_frame_odom_){
                    new_frame_odom_msg_.header.stamp = msg_in->header.stamp;
                    new_frame_odom_msg_.twist = msg_in->twist;
                    tf::poseTFToMsg(corrected,new_frame_odom_msg_.pose.pose);
                    output_pub_.publish(new_frame_odom_msg_);
                }
            }
            else
                corrected = odom;
            tf_.sendTransform(tf::StampedTransform(corrected, msg_in->header.stamp, odom_parent_link_, odom_child_link_));
        }
        if(publish_child_link)
            tf_.sendTransform(tf::StampedTransform(offset_tf_, msg_in->header.stamp, odom_child_link_, child_link_));

    }

private:

    std::string odom_topic_,new_frame_odom_topic_;
    std::string odom_parent_link_, odom_child_link_;
    std::string child_link_;
    bool publish_child_link, convert_odom_to_tf_, use_odom_frame_offset_,publish_new_frame_odom_,  initialized_;
    tf::Transform offset_tf_, new_odom_frame_;
    nav_msgs::Odometry new_frame_odom_msg_;
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



