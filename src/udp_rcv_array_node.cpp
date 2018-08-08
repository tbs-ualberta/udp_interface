// UDP socket includes
#include <data_transmission/data_transmission.h>
#include <string.h>

// ROS includes
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_rcv_array");
  ros::NodeHandle n;

  ros::Publisher data_pub =
      n.advertise<std_msgs::Float64MultiArray>("data", 1);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  ros::Rate loop_rate(rate_hz);
  n.getParam("udp_rcv_array/rate", rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_rcv_array/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7778;
  if (!n.getParam("udp_rcv_array/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int len_rcvbuf = 10;
  n.getParam("udp_rcv_array/len_rcvbuf", len_rcvbuf);
  int len_array = 6;
  n.getParam("udp_rcv_array/len_array", len_array);

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  transmission.init_transmission(ip_local_scp, port_local_si, len_rcvbuf);
  char message[1024];

  while (ros::ok()) {
    transmission.listen(message, sizeof(double) * len_array);
    // Convert to double
    double *message_dp = (double *)message;

    std_msgs::Float64MultiArray msg_array;
    // Clear array
    msg_array.data.clear();
    // Populate array
    for (int i = 0; i < len_array; i++) {
      msg_array.data.push_back(message_dp[i]);
    }

    data_pub.publish(msg_array);

    ros::spinOnce();
    loop_rate.sleep();
  }

  transmission.close_transmission();
}
