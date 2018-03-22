// UDP socket includes
#include <data_transmission.h>
#include <string.h>

// ROS includes
#include "ros/ros.h"
#include <std_msgs/Int32MultiArray.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_interface");
  ros::NodeHandle n;

  // --- Obtain parameters ---
  int rate_hz = 1000;
  n.getParam("udp_snd_testdelay/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_snd_testdelay/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7777;
  if (!n.getParam("udp_snd_testdelay/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  string ip_remote_st = "0.0.0.0";
  if (!n.getParam("udp_snd_testdelay/ip_rem", ip_remote_st)) {
    ROS_ERROR("Parameter \"ip_rem\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_remote_si = 7777;
  if (!n.getParam("udp_snd_testdelay/port_rem", port_remote_si)) {
    ROS_ERROR("Parameter \"port_rem\" not provided. Exiting.");
    ros::shutdown();
  }

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  char ip_remote_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  strcpy(ip_remote_scp, ip_remote_st.c_str());
  transmission.init_transmission(ip_local_scp, port_local_si, ip_remote_scp,
                                 port_remote_si);

  while (ros::ok()) {

    // --- Construct the message ---
    ros::Time time_now = ros::Time::now();
    double time_now_d = double(time_now.sec) + double(time_now.nsec) * 1e-9;
    ROS_DEBUG_STREAM_THROTTLE(1, "time_now_d = " << time_now_d);
    transmission.send(&time_now_d, 1);

    ros::spinOnce();
    loop_rate.sleep();
  }

  transmission.close_transmission();
}
