// UDP socket includes
#include <data_transmission/data_transmission.h>
#include <string.h>

// ROS includes
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_interface");
  ros::NodeHandle n;

  ros::Publisher data_pub = n.advertise<geometry_msgs::WrenchStamped>("ft", 1);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  ros::Rate loop_rate(rate_hz);
  n.getParam("udp_rcv_wrench/rate", rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_rcv_wrench/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7778;
  if (!n.getParam("udp_rcv_wrench/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int rcvbuf_len = 10;
  n.getParam("udp_rcv_testdelay/rcvbuf_len", rcvbuf_len);

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  transmission.init_transmission(ip_local_scp, port_local_si, rcvbuf_len);

  char message[1024];
  while (ros::ok()) {
    // TODO Make the below non-blocking (small timeout)
    transmission.listen(message, sizeof(double) * 6);
    // Convert to double
    double *message_dp = (double *)message;

    // --- Assemble & publish the message ---
    geometry_msgs::WrenchStamped msg_wrench;
    msg_wrench.header.stamp = ros::Time::now();
    msg_wrench.header.frame_id = "base_link";
    msg_wrench.wrench.force.x = message_dp[0];
    msg_wrench.wrench.force.y = message_dp[1];
    msg_wrench.wrench.force.z = message_dp[2];
    msg_wrench.wrench.torque.x = message_dp[3];
    msg_wrench.wrench.torque.y = message_dp[4];
    msg_wrench.wrench.torque.z = message_dp[5];
    data_pub.publish(msg_wrench);

    ros::spinOnce();
    loop_rate.sleep();
  }
  transmission.close_transmission();
}
