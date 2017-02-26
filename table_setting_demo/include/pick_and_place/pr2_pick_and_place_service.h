#ifndef PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE
#define PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "table_setting_demo/pick_and_place.h"
#include "actionlib/client/simple_action_client.h"
#include "arm_navigation_msgs/MoveArmAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "tf/transform_listener.h"
// #include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/pick_and_place_state.h"
#include "table_setting_demo/pick_and_place_stop.h"

namespace pr2 {

typedef enum STATE {
  APPROACHING = 0,
  PICKING,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL,
  IDLE
} STATE_t;

struct PickPlaceGoal {
  arm_navigation_msgs::MoveArmGoal pick_pose;
  arm_navigation_msgs::MoveArmGoal place_pose;
};

typedef struct Point {
  double x,y,z,w;
}__attribute__((packed)) Point_t;

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef arm_navigation_msgs::MoveArmGoal MoveArmGoal_t;

class Gripper {
 public:
  Gripper();
  virtual ~Gripper();

  virtual void Open();
  virtual void Close();
 private:
  GripperClient *gripper_client_;
};

class PickPlace {
 public:
  PickPlace(std::string arm);
  virtual ~PickPlace();

  bool PickAndPlaceObject(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlacecheck(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res);
  bool PickAndPlaceState(
    table_setting_demo::pick_and_place_state::Request &req,
    table_setting_demo::pick_and_place_state::Response &res);
  bool PickAndPlaceStop(
    table_setting_demo::pick_and_place_stop::Request &req,
    table_setting_demo::pick_and_place_stop::Response &res);
  void PostParameters();
  void CalibrateObjects();
  void ReadCalibration(std::string filename);
  MoveArmGoal_t GetArmPoseFromPoints(
    std::string frame_id,
    std::string link,
    Point_t position,
    Point_t orientation);
  void SaveCalibration(std::string filename);
  void PickAndPlaceImpl(std::string object);

 private:
  bool SendGoal(MoveArmGoal_t goal);
  MoveArmGoal_t GetArmPoseGoal();

  ros::NodeHandle nh_;
  std::vector<std::string> objects_;
  std::vector<std::string> static_objects_;
  std::vector<std::string> dynamic_objects_;
  std::string arm_;
  std::map<std::string, PickPlaceGoal> object_goal_map_;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_;
  Gripper r_gripper_;
  uint32_t state_;
  bool stop;
  boost::shared_ptr<boost::thread> work_thread;
};
}
#endif  // PICK_AND_PLACE_PR2_PICK_AND_PLACE_SERVICE
