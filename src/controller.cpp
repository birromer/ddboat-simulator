#include "ros/ros.h"

#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"

#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#include "visualization_msgs/Marker.h"

#include "tf/tf.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <math.h>

double robot_x, robot_y, robot_th, robot_sp; // robot's state (x, y, heading, speed)
double target_x, target_y;  // position of the local objective

void robot_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  robot_x = msg->pose.position.x;
  robot_y = msg->pose.position.y;
  robot_th = tf::getYaw(msg->pose.orientation);
  robot_sp = 1.0;

  ROS_INFO("Received robot's state at position: [%f] [%f] [%f], with speed [%f]", robot_x, robot_y, robot_th, robot_sp);
}

void target_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  target_x = msg->pose.position.x;
  target_y = msg->pose.position.y;

  ROS_INFO("Received target at position: [%f] [%f]", target_x, target_y);
}

int main(int argc, char **argv) {
  // initialization and instantiation of the node
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  ros::Rate loop_rate(25);  // 25Hz frequency

  ros::Publisher z_u1_pub = n.advertise<std_msgs::Float32>("z_u1", 1000);
  ros::Publisher z_u2_pub = n.advertise<std_msgs::Float32>("z_u2", 1000);

  ros::Subscriber robot_sub = n.subscribe("state_boat", 1000, robot_Callback);
  ros::Subscriber target_sub = n.subscribe("/target", 1000, target_Callback);

  double x1, x2, x3, x4, x_t, y_t;
  double u1, u2, e_angle, e_dist, angle_boat_trgt, angle_turr_trgt;

  std_msgs::Float32 msg_z_u1, msg_z_u2;

  std::string ns_boat = ros::this_node::getNamespace().substr(1,5);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  while (ros::ok()) {
    // copy vars for them not to change inside loop
    x1 = robot_x;
    x2 = robot_y;
    x3 = robot_th;
    x4 = robot_sp;
    x_t = target_x;
    y_t = target_y;

    // ====================== PD control ======================
    //TODO: change that control for something that works on two propulsors

//    angle_boat_trgt = atan2(x2-y_t, x1-x_t);
//    // error is the angle between the mais axis of the boat and the target
//    e_angle = angle_boat_trgt - x3;
//    ROS_INFO("Current angle error: [%f]", e_angle);
//
//    // deal with the discontinuity problem with sawtooth
//    e_angle = 2*atan(tan(e_angle/2));
//
//    // distance to the target, desired is 5m
//    e_dist = sqrt(pow(x1 - x_t, 2) + pow(x2 - y_t, 2));
//    ROS_INFO("Current distance between robot and target: [%f]", e_dist);
//
//    if (e_angle > M_PI/4) {
//      u1 = 0-10;  // turn right
//    } else {
//      u1 = 0.1*(e_angle+M_PI);
//    }

    u1 = 10;
    u2 = 10;

    msg_z_u1.data = u1;
    z_u1_pub.publish(msg_z_u1);

    msg_z_u2.data = u2;
    z_u2_pub.publish(msg_z_u2);

    ROS_INFO("Sent u1 [%f] and u2 [%f]", u1, u2);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
