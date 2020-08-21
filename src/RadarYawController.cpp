#include "ros/ros.h"
#include "ti_mmwave_rospkg/RadarScan.h"
#include "ti_mmwave_rospkg/RadarCube.h"
#include "std_msgs/Float64.h"
#include <string>
// #include "NumCpp.hpp"
#include <math.h>

#define STEP_INCREMENT 15
// #define STEPS_PER_HALF_REV 6
// #define STEP_INCREMENT 180 / STEPS_PER_HALF_REV

// ros::Publisher radar_yaw_position;
ros::Publisher radar_yaw_cmd;
float current_yaw = M_PI;
int current_yaw_idx = 180 / STEP_INCREMENT / 2;
float current_yaw_dir = 1;

// nc::NdArray<float> positions = nc::arange<float>(90, 270, (int)(180 / STEP_INCREMENT) + 1);
#define POS_SIZE 12
float positions[] = { 90., 105., 120., 135., 150., 165., 180., 195., 210., 225., 240., 255., 270. };
// auto positions = nc::hstack( { nc::arange<float>(PI, PI + PI / 2 + 1, STEP_INCREMENT / 2), 
//                                nc::arange<float>(PI + PI / 2, PI - PI / 2, STEPS_PER_HALF_REV), 
//                                nc::arange<float>(PI - PI / 2, PI, STEPS_PER_HALF_REV / 2), });

void set_position_deg(float new_yaw) {
  std_msgs::Float64 new_yaw_rad;
  new_yaw_rad.data = new_yaw * M_PI / 180;  // deg to rad
  ROS_INFO("Moving to new pos: %f", new_yaw);
  radar_yaw_cmd.publish(new_yaw_rad);
  current_yaw = new_yaw_rad.data;
}

void set_next_pos() {
  current_yaw_idx += current_yaw_dir;
  if (current_yaw_idx > POS_SIZE) {
    current_yaw_idx = POS_SIZE - 1;
    current_yaw_dir = -1;
  }
  if (current_yaw_idx < 0) {
    current_yaw_idx = 1;
    current_yaw_dir = 1;
  }
  set_position_deg(positions[current_yaw_idx]);
}

void move_to_new_pos(const ti_mmwave_rospkg::RadarCube &RadarScan) {
  ros::Duration(0.06).sleep();  // wait for the radar to perform the chirps and go into transfer state. 
  set_next_pos(); // move radar while in transfer state
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_yaw_controller");
  ros::NodeHandle n;
  ROS_INFO("Init");
  std::string controller_cmd_topic = argv[1];  // /arm_control/radar_yaw_position_controller/command
  radar_yaw_cmd = n.advertise<std_msgs::Float64>(controller_cmd_topic, 10);
  set_position_deg(180);
  ros::Subscriber sub = n.subscribe("/ti_mmwave/radar_cube", 10, move_to_new_pos);
  // radar_yaw_position = n.advertise<float>("/ti_mmwave/radar_yaw", 100);
  ros::spin();
  ros::waitForShutdown();

  return 0;
}