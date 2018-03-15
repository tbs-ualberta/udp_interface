// UDP socket includes
#include <data_transmission.h>
#include <string.h>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Quaternion to_quaternion(double x, double y, double z) {

  double cz = cos(z * 0.5);
  double sz = sin(z * 0.5);
  double cy = cos(y * 0.5);
  double sy = sin(y * 0.5);
  double cx = cos(x * 0.5);
  double sx = sin(x * 0.5);

  geometry_msgs::Quaternion q;
  q.w = cz * cy * cx + sz * sy * sx;
  q.x = cz * sy * cx - sz * cy * sx;
  q.y = cz * cy * sx + sz * sy * cx;
  q.z = sz * cy * cx - cz * sy * sx;
  return q;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "udp_interface");
  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1);

  // --- Obtain parameters ---
  int rate_hz = 1000;
  ros::Rate loop_rate(rate_hz);
  n.getParam("udp_rcv_posee2q/rate", rate_hz);
  string ip_local_st = "0.0.0.0";
  if (!n.getParam("udp_rcv_posee2q/ip_loc", ip_local_st)) {
    ROS_ERROR("Parameter \"ip_loc\" not provided. Exiting.");
    ros::shutdown();
  }
  int port_local_si = 7778;
  if (!n.getParam("udp_rcv_posee2q/port_loc", port_local_si)) {
    ROS_ERROR("Parameter \"port_loc\" not provided. Exiting.");
    ros::shutdown();
  }

  // --- Setup the transmission ---
  data_transmission transmission;
  char ip_local_scp[1024];
  strcpy(ip_local_scp, ip_local_st.c_str());
  transmission.init_transmission(ip_local_scp, port_local_si);

  char message[1024];
  while (ros::ok()) {
    // TODO Make the below non-blocking (small timeout)
    transmission.listen(message, sizeof(double) * 6);
    // Convert to double
    double *message_dp = (double *)message;

    ros::Time time_now = ros::Time::now();

    // --- Assemble the pose message ---
    geometry_msgs::PoseStamped msg_pose;
    msg_pose.header.stamp = time_now;
    msg_pose.header.frame_id = "odom";
    msg_pose.pose.position.x = message_dp[0];
    msg_pose.pose.position.y = message_dp[1];
    msg_pose.pose.position.z = message_dp[2];
    msg_pose.pose.orientation =
        to_quaternion(message_dp[3], message_dp[4], message_dp[5]);
    pose_pub.publish(msg_pose);

    // --- Assemble & broadcast the frame transformation ---
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(message_dp[0], message_dp[1], message_dp[2]));
    tf::Quaternion q;
    q.setEuler(message_dp[4], message_dp[3], message_dp[5]);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, time_now, "base_link", "odom"));

    ros::spinOnce();
    loop_rate.sleep();
  }
  transmission.close_transmission();
}
