#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include <math.h>
#include "visualization_msgs/Marker.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define P1 1
#define P2 1
#define P3 1

#define DATA_FROM_SIM true

double cmd_r, cmd_l;

void cmd_r_Callback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("received command right: [%f]", msg->data);
  cmd_r = msg->data;
}

void cmd_l_Callback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("received command left: [%f]", msg->data);
  cmd_l = msg->data;
}

void integration_euler(double &x1, double &x2, double &x3, double &x4, double u1, double u2, double dt) {
  x1 = x1 + dt * (x4*cos(x3));
  x2 = x2 + dt * (x4*sin(x3));
  x3 = x3 + dt * (P1*(u1-u2));
  x4 = x4 + dt * (P2*(u1+u2) + P3*x4*abs(x4));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "boat_simulator");  // initialization of the node
  ros::NodeHandle n;  // access to the node

  ros::Rate loop_rate(25);  // 25Hz frequency

  double dt = 0.005;  // time step

  double u1 = cmd_r;
  double u2 = cmd_l;

  double x1, x2, x3, x4;

  // read initial values from the launcher
  n.param<double>("pos_x_init", x1, 0);
  n.param<double>("pos_y_init", x2, 0);
  n.param<double>("pos_th_init", x3, 0);
  n.param<double>("pos_v_init", x4, 0);

  // subscriber to z inputs from outside
  ros::Subscriber cmd_r_sub = n.subscribe("z_u1", 1000, cmd_r_Callback);
  ros::Subscriber cmd_l_sub = n.subscribe("z_u2", 1000, cmd_l_Callback);

  // published of the robot's state
  ros::Publisher state_pub = n.advertise<geometry_msgs::PoseStamped>("state_boat", 1000);

//  // publisher for the 3d visualization
//  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // transform broadcaster between the map and the robot
  std::string ns = ros::this_node::getNamespace();

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = ns.substr(1,5);

  while (ros::ok()) {
    u1 = cmd_r;
    u2 = cmd_l;

    if (DATA_FROM_SIM == false) {
      // evolve model
      integration_euler(x1, x2, x3, x4, u1, u2, dt);

      // publish simulation
      geometry_msgs::PoseStamped msg;
      msg.pose.position.x = x1;
      msg.pose.position.y = x2;
      msg.pose.position.z = 0;  // assumed to be constant 0 for now

      tf::Quaternion q;
      q.setRPY(0, 0, x3);  // roll, pitch, yaw - theta normally pitch, but i think it means yaw now
      tf::quaternionTFToMsg(q, msg.pose.orientation);

      msg.header.stamp = ros::Time::now();  // in order to better show the related graphs
      msg.header.frame_id = "map";

      state_pub.publish(msg);

//      visualization_msgs::Marker marker;
//      marker.header.frame_id = "map";
//      marker.header.stamp = ros::Time::now();
//      marker.ns = ros::this_node::getNamespace();
//      marker.id = 0;
//      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//      marker.action = visualization_msgs::Marker::ADD;
//      marker.pose.position.x = x1;
//      marker.pose.position.y = x2;
//      marker.pose.position.z = 0;
//      tf::quaternionTFToMsg(q, marker.pose.orientation);
//      marker.scale.x = 1;
//      marker.scale.y = 1;
//      marker.scale.z = 1;
//      marker.color.a = 1.0; // alpha = transparence
//      marker.color.r = 1.0;
//      marker.color.g = 1.0;
//      marker.color.b = 1.0;
//      marker.mesh_resource = "package://tp3/meshs/boat.dae";
//      vis_pub.publish(marker);

      transformStamped.header.stamp = ros::Time::now();
      transformStamped.transform.translation.x = x1;
      transformStamped.transform.translation.y = x2;
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      br.sendTransform(transformStamped);
    } else {
      // get simulation data
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
