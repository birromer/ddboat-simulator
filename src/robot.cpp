#include "ros/ros.h"
#include "std_srvs/Trigger.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ddboat_sim/State.h"

#include <math.h>

#define P1 1
#define P2 1
#define P3 1

#define DATA_FROM_SIM false

double cmd_r, cmd_l;
double lx, ly;               // gps data
double compass;              // compass data
double acc_x, acc_y, acc_z;  // accelerometer data
double enc_left, enc_right;  // encoders data

double prev_x1=0, prev_x2=0;

void cmd_r_Callback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("received command right: [%f]", msg->data);
  cmd_r = msg->data;
}

void cmd_l_Callback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("received command left: [%f]", msg->data);
  cmd_l = msg->data;
}

/*  Data being expected from the boat
** Un topic pour le GPS (array de deux valeurs lx, ly)
** Un topic pour le compass (un float)
** Un topic pour accéléromètre (array de 3 valeurs)
** Un topic pour les encodeur (array de 2 valeurs)
 */

void gps_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ROS_INFO("received gps data: lx [%f], ly [%f]", msg->data[0], msg->data[1]);
  lx = msg->data[0];
  ly = msg->data[1];
}

void compass_Callback(const std_msgs::Float32::ConstPtr& msg) {
  ROS_INFO("received compass data: [%f]", msg->data);
  compass = msg->data;
}

void acc_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ROS_INFO("received accelerometer data: x [%f], y [%f], z [%f]", msg->data[0], msg->data[1], msg->data[2]);
  acc_x = msg->data[0];
  acc_y = msg->data[1];
  acc_z = msg->data[2];
}

void encoder_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ROS_INFO("received encoder data: left [%f], right [%f]", msg->data[0], msg->data[1]);
  enc_left  = msg->data[0];
  enc_right = msg->data[1];
}

void integration_euler(double &x1, double &x2, double &x3, double &x4, double u1, double u2, double dt) {
  std::cout << "x1: " << x1 << "| x2: "  << x2 << " | x3: " << x3 << " | x4 " << x4 << std::endl;
  x1 = x1 + dt * (x4*cos(x3));
  x2 = x2 + dt * (x4*sin(x3));
  std::cout << "x1: " << x1 << "| x2: "  << x2 << " | x3: " << x3 << " | x4 " << x4 << std::endl;
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

  ros::Subscriber gps_sub = n.subscribe("gps", 1000, gps_Callback);
  ros::Subscriber compass_sub = n.subscribe("compass", 1000, compass_Callback);
  ros::Subscriber acc_sub = n.subscribe("acc", 1000, acc_Callback);
  ros::Subscriber encoder_sub = n.subscribe("encoder", 1000, encoder_Callback);

  // published of the robot's state
  ros::Publisher state_pub = n.advertise<ddboat_sim::State>("state_boat", 1000);

  while (ros::ok()) {
    std::cout << "inicio do loop" << std::endl;
    u1 = cmd_r;
    u2 = cmd_l;

    if (DATA_FROM_SIM == false) { // evolve model
      std::cout << "entrei no caso com simulacao" << std::endl;
      integration_euler(x1, x2, x3, x4, u1, u2, dt);

    } else { // get simulation data
      std::cout << "entrei no caso sem simulacao" << std::endl;

      double vit = sqrt(pow(x1 - prev_x1, 2) + pow(x2-prev_x2,2));

      x1 = lx;
      x2 = ly;
      x3 = compass;
      x4 = vit;

      prev_x1 = x1;
      prev_x2 = x2;

    }

    // publish simulation
    ddboat_sim::State state_msg;

    state_msg.x1 =  x1;
    state_msg.x2 =  x2;
    state_msg.x3 =  x3;
    state_msg.x4 =  x4;

    state_pub.publish(state_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
