#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <eigen3/Eigen/Eigen>

namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh)
    : nh_(nh) {

  // initialize subscribers
  odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>(
      "odometry", 10, &PX4_Realsense_Bridge::odomCallback, this);
  // publishers
  mavros_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
  mavros_system_status_pub_ =
      nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);

  last_callback_time = ros::Time::now();

  status_mutex_.reset(new std::mutex);
  worker_ = std::thread(&PX4_Realsense_Bridge::publishSystemStatus, this);


};

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() { }


void PX4_Realsense_Bridge::odomCallback(const nav_msgs::Odometry& msg) {

  // publish odometry msg
  nav_msgs::Odometry output = msg;
  output.header.frame_id = "odom";
  output.child_frame_id = "base_link";
  
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_cov(output.pose.covariance.data());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_cov(output.pose.covariance.data());
  pose_cov(0, 0) == 0.01;
  pose_cov(1, 1) == 0.01;
  pose_cov(2, 2) == 0.01;

  pose_cov(3, 3) == 0.01;
  pose_cov(4, 4) == 0.01;
  pose_cov(5, 5) == 0.01;

  twist_cov(3, 3) == 0.01;
  twist_cov(4, 4) == 0.01;
  twist_cov(5, 5) == 0.01;

  twist_cov(3, 3) == 0.01;
  twist_cov(4, 4) == 0.01;
  twist_cov(5, 5) == 0.01;

  mavros_odom_pub_.publish(output);

  flag_first_pose_received = true;

  { // lock mutex
    std::lock_guard<std::mutex> status_guard(*(status_mutex_));

    last_system_status_ = system_status_;

    system_status_ = MAV_STATE::MAV_STATE_ACTIVE;

    // publish system status immediately if it changed
    if( last_system_status_ != system_status_ )
    {
      mavros_msgs::CompanionProcessStatus status_msg;

      status_msg.header.stamp = ros::Time::now();
      status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

      status_msg.state = (int)system_status_;

      mavros_system_status_pub_.publish(status_msg);
    }  

  last_callback_time = ros::Time::now();
    
  }
}


void PX4_Realsense_Bridge::publishSystemStatus(){
  

  while(ros::ok()){
    
    ros::Duration(1).sleep();

    if(flag_first_pose_received == true) { // only send heartbeat if we receive pose estimates at all

      // check if we received an recent update
      // otherwise let the companion computer restart
      if( (ros::Time::now()-last_callback_time) > ros::Duration(0.5) ){
        ROS_WARN_STREAM("Stopped receiving data from VINS");
        system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
      }

      mavros_msgs::CompanionProcessStatus status_msg;

      status_msg.header.stamp = ros::Time::now();
      status_msg.component = 197;  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    
      { // lock mutex
        std::lock_guard<std::mutex> status_guard(*(status_mutex_));

        status_msg.state = (int)system_status_;

        mavros_system_status_pub_.publish(status_msg);
      }
    }
  }

}

}
