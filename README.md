A ROS package providing a data interface to arbitrary non-ROS software and
hardware interfaces via the UDP protocol. Implemented nodes can handle various
message structures (see below).   

The package is still under development and therefore rather unstable.

#### Nodes:
- **udp_rcv_posee2q**:
  - Receives a 6-element array ([posx posy posz roll pitch yaw]), converts the data to ``geometry_msgs/PoseStamped``-messages and publishes to topic "pose". The node also broadcasts a transformation from "base_link" (assumed fixed) to "odom".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
    - *len_rcvbuf*: Length of the UDP socket's receive buffer (SO_RCVBUF)
- **udp_rcv_wrench**:
  - Receives a 6-element array ([fx fy fz tx ty tz]), converts the data to ``geometry_msgs/WrenchStamped``-messages and publishes to topic "ft".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
    - *len_rcvbuf*: Length of the UDP socket's receive buffer (SO_RCVBUF)
- **udp_rcv_array**:
  - Receives an array of doubles of arbitrary length (provided in parameters), converts the data to ``std_msgs/Float64MultiArray``-messages and publishes to topic "data".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
    - *len_rcvbuf*: Length of the UDP socket's receive buffer (SO_RCVBUF)
    - *len_array*: Length of the received data array
  For instace, to recieve an array of double of size 8, one can use the command below:
  ```bash
  rosrun udp_interface udp_rcv_array _ip_loc:="192.1.1.1" _port_loc:=43000 _len_array:=8
  ```
- **udp_snd_wrench**:
  - Sends a 6-element array ([fx fy fz tx ty tz]) converted from ``geometry_msgs/WrenchStamped``-messages obtained from topic "ft".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_rem*: Remote IP address ["0.0.0.0"] (``String``)
    - *port_rem*: Remote port (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
- **udp_snd_array**:
  - Sends an array of doubles of arbitrary length (provided in parameters), converts the data to ``std_msgs/Float64MultiArray``-messages obtained from topic "data".
  - *Parameters*:
    - *rate*: Sampling rate [Hz] (``int``)
    - *ip_rem*: Remote IP address ["0.0.0.0"] (``String``)
    - *port_rem*: Remote port (``int``)
    - *ip_loc*: Local IP address ["0.0.0.0"] (``String``)
    - *port_loc*: Local port (``int``)
    - *len_array*: Length of the sent data array

#### Dependencies:
- https://github.com/tbs-ualberta/data_transmission.git  

#### Installation:
1. Install [data transmission library](https://github.com/tbs-ualberta/data_transmission.git)
2. ``cd ~/catkin_ws/src``  
   ``git clone https://github.com/tbs-ualberta/udp_interface.git``  
   ``cd ../``  
   ``catkin_make``
