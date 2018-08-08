// UDP socket includes
#include <data_transmission/data_transmission.h>
#include <string.h>

// ROS includes
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

// Globals
data_transmission g_transmission;
int g_len_array = 6;

void dataCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  double data[g_len_array];
  for (int i = 0; i < g_len_array; i++) {
    data[i] = msg->data.at(i);
  }
  g_transmission.send(data, g_len_array);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_snd_array");
  ros::NodeHandle n;

  ros::Subscriber data_sub = n.subscribe("data", 1, dataCallback);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  n.getParam("udp_snd_array/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_snd_array/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7777;
  if (!n.getParam("udp_snd_array/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  string ip_remote_st = "0.0.0.0";
  if (!n.getParam("udp_snd_array/ip_rem", ip_remote_st)) {
    ROS_ERROR("Parameter \"ip_rem\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_remote_si = 7777;
  if (!n.getParam("udp_snd_array/port_rem", port_remote_si)) {
    ROS_ERROR("Parameter \"port_rem\" not provided. Exiting.");
    ros::shutdown();
  }
  n.getParam("udp_rcv_array/len_array", g_len_array);

  // --- Setup the transmission ---
  char ip_local_scp[1024];
  char ip_remote_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  strcpy(ip_remote_scp, ip_remote_st.c_str());
  g_transmission.init_transmission(ip_local_scp, port_local_si, ip_remote_scp,
                                   port_remote_si);

  ros::spin();

  g_transmission.close_transmission();
}
