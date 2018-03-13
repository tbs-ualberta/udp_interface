A ROS package providing a data interface to arbitrary non-ROS software and
hardware interfaces via the UDP protocol. Implemented nodes can handle various
message structures (see below).   

The package is still under development and thus rather unstable.

#### Nodes:
- **udp_rcv_posee2q**:
  - Receives a 6-element array ([posx posy posz roll pitch yaw]), converts the data to ``geometry_msgs/PoseStamped``-messages and publishes to topic "pose". The node also broadcasts a transformation from "base_link" (assumed fixed) to "odom".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
- **udp_rcv_wrench**:
  - Receives a 6-element array ([fx fy fz tx ty tz]), converts the data to ``geometry_msgs/WrenchStamped``-messages and publishes to topic "ft".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
- **udp_snd_wrench**:
  - Sends a 6-element array ([fx fy fz tx ty tz]) converted from ``geometry_msgs/WrenchStamped``-messages obtained from topic "ft".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_rem*: Remote IP address ["0.0.0.0"] (``String``)
    - *port_rem*: Remote port (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)

#### Dependencies:
- https://github.com/tbs-ualberta/data_transmission.git  

#### Installation:
1. Install [data transmission library](https://github.com/tbs-ualberta/data_transmission.git)
2. ``cd ~/catkin_ws/src``  
   ``git clone https://github.com/tbs-ualberta/udp_interface.git``  
   ``cd ../``  
   ``catkin_make``
