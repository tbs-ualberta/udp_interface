// UDP socket includes
#include <data_transmission.h>
#include <string.h>

// ROS includes
#include <std_msgs/Float64MultiArray.h>
//#include <rosbag/bag.h>
#include "ros/ros.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_interface");
  ros::NodeHandle n;

  ros::Publisher data_pub =
      n.advertise<std_msgs::Float64MultiArray>("testdelay", 1);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  ros::Rate loop_rate(rate_hz);
  n.getParam("udp_rcv_testdelay/rate", rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_rcv_testdelay/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7778;
  if (!n.getParam("udp_rcv_testdelay/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  transmission.init_transmission(ip_local_scp, port_local_si);
  char message[1024];

  // --- Setup rosbag ---
  // rosbag::Bag bag;
  // bag.open("testdelay.bag", rosbag::bagmode::Write);
  while (ros::ok()) {
    // TODO Make the below non-blocking (small timeout)
    transmission.listen(message, 8);
    // Convert to double
    double *message_dp = (double *)message;
    // ROS_INFO("%f", (*message_dp));
    ROS_DEBUG_STREAM_THROTTLE(1, "*message_dp = " << (*message_dp));
    ros::Time time_now = ros::Time::now();
    double time_now_d = double(time_now.sec) + double(time_now.nsec) * 1e-9;
    std_msgs::Float64MultiArray msg_array;
    // Clear array
    msg_array.data.clear();
    msg_array.data.push_back(message_dp[0]);
    msg_array.data.push_back(time_now_d);

    // bag.write("times", time_now, message_final_ip);
    data_pub.publish(msg_array);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // bag.close();
  transmission.close_transmission();
}
