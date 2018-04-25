// UDP socket includes
#include <data_transmission/data_transmission.h>
#include <string.h>

// ROS includes
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

// Globals
data_transmission _transmission;

void dataCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  double data[6];
  data[0] = msg->wrench.force.x;
  data[1] = msg->wrench.force.y;
  data[2] = msg->wrench.force.z;
  data[3] = msg->wrench.torque.x;
  data[4] = msg->wrench.torque.y;
  data[5] = msg->wrench.torque.z;
  _transmission.send(data, 6);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_interface");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ft", 1, dataCallback);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  n.getParam("udp_snd_wrench/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_snd_wrench/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7777;
  if (!n.getParam("udp_snd_wrench/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  string ip_remote_st = "0.0.0.0";
  if (!n.getParam("udp_snd_wrench/ip_rem", ip_remote_st)) {
    ROS_ERROR("Parameter \"ip_rem\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_remote_si = 7777;
  if (!n.getParam("udp_snd_wrench/port_rem", port_remote_si)) {
    ROS_ERROR("Parameter \"port_rem\" not provided. Exiting.");
    ros::shutdown();
  }

  // --- Setup the transmission ---
  char ip_local_scp[1024];
  char ip_remote_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  strcpy(ip_remote_scp, ip_remote_st.c_str());
  _transmission.init_transmission(ip_local_scp, port_local_si, ip_remote_scp,
                                  port_remote_si);

  // Loop until user aborts (Ctrl+C)
  ros::spin();

  _transmission.close_transmission();
}
