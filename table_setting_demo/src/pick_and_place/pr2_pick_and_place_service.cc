#include "pick_and_place/pr2_pick_and_place_service.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/ObjectTransformation.h"
#include "log.h"
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int getch() {
  static termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

namespace pr2 {
//Open the gripper
Gripper::Gripper(){

  //Initialize the client for the Action interface to the gripper controller
  //and tell the action client that we want to spin a thread by default
  gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

  //wait for the gripper action server to come up 
  while(!gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action"
             "action server to come up");
  }
}

Gripper::~Gripper(){
  delete gripper_client_;
}

void Gripper::Open(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = 80.0;  // Do not limit effort (negative)
  
  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else
    ROS_INFO("The gripper failed to open.");
}

//Close the gripper
void Gripper::Close(){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = 30.0;  // Close gently
  
  ROS_INFO("Sending squeeze goal");
  gripper_client_->sendGoal(squeeze);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper closed!");
  else
    ROS_INFO("The gripper failed to close.");
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

PickPlace::PickPlace(std::string arm) /* : //TODO JB: move_arm_("move_right_arm",true) */ {
  arm_ = arm;
  const char *dynamic_object_str[] = {
    // "cup",
    // "bowl",
    // "soda",
    "fork",
    "spoon",
    "knife"
  };
  const char *static_object_str[] = {
    "cup",
    "bowl",
    "soda",
    "neutral",
    "placemat",
    "wineglass",
    "plate"
  };
  const char *object_str[] = {
    "neutral",
    "placemat",
    "cup",
    "plate",
    "fork",
    "spoon",
    "knife",
    "bowl",
    "soda",
    "wineglass"
  };
  objects_ = std::vector<std::string>(object_str,
    object_str + sizeof(object_str) / sizeof(char*));

  static_objects_ = std::vector<std::string>(static_object_str,
    static_object_str + sizeof(static_object_str) / sizeof(char*));

  dynamic_objects_ = std::vector<std::string>(dynamic_object_str,
    dynamic_object_str + sizeof(dynamic_object_str) / sizeof(char*));

  for (uint32_t i = 0; i < objects_.size(); ++i) {
    printf("Object: %s\n", objects_[i].c_str());
  } 

  // SET STATE
  state_ = IDLE;
} 

PickPlace::~PickPlace() {}

void PickAndPlaceThread(PickPlace *manipulation, std::string object) {
  manipulation->PickAndPlaceImpl(object);
}

void TransformPoseLocalToWorld(
  geometry_msgs::PoseStamped &input,
  geometry_msgs::PoseStamped &output,
  geometry_msgs::TransformStamped transform) {
  output.pose.position.x = input.pose.position.x + transform.transform.translation.x;
  output.pose.position.y = input.pose.position.y + transform.transform.translation.y;
  output.pose.position.z = input.pose.position.z + transform.transform.translation.z;

  output.pose.orientation = input.pose.orientation;
}

void TransformPoseWorldToLocal(
  geometry_msgs::PoseStamped &input,
  geometry_msgs::PoseStamped &output,
  geometry_msgs::TransformStamped transform) {
  output.pose.position.x = input.pose.position.x - transform.transform.translation.x;
  output.pose.position.y = input.pose.position.y - transform.transform.translation.y;
  output.pose.position.z = input.pose.position.z - transform.transform.translation.z;

  output.pose.orientation = input.pose.orientation;
}

void PickPlace::PickAndPlaceImpl(std::string object) {
  printf("Picking up Object: %s\n", object.c_str());
  // check if dyanamic or static object
  if (stop)
    return;
  table_setting_demo::object_position pos_msg;
  table_setting_demo::ObjectTransformation pose_msg;
  bool dynamic = true;
  for (int i = 0; i < static_objects_.size(); ++i) {
    if (object == static_objects_[i]) {
      dynamic = false;
      break;
    }
  }
  if (dynamic) {
    ROS_INFO("Object Is dynamic");
  } else {
    ROS_INFO("Object Is Static");
  }
  state_ = NEUTRAL;
  r_gripper_.Open();
  state_ = APPROACHING;
  // Move to Neutral Start
//------
  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // N/A?!  
//------
  
  if (stop)
    return;
//------
  /* //TODO JB: if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    return;
  } */
  if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    return;
  } 
//------
  if (stop)
    return;
  // Move to Object Pick location

  // Check if Object is dynamic or static
  if (dynamic) {
    // request object tracked position
    pos_msg.request.object_id = object;
    if (!ros::service::call("qr_get_object_position", pos_msg)) {
      ROS_ERROR("Service: [%s] not available!", "qr_get_object_position");
    }
    // Request object 3D transform
    if (pos_msg.response.position.size() > 0) {
      pose_msg.request.x = pos_msg.response.position[0];
      pose_msg.request.y = pos_msg.response.position[1];
      pose_msg.request.w = pos_msg.response.position[2];
      pose_msg.request.h = pos_msg.response.position[3];
      pose_msg.request.object = object;

      if (!ros::service::call("object_transformation", pose_msg)) {
        ROS_ERROR("Service: [%s] not available!", "object_transformation");
      }
    } else {
      pose_msg.response.transform.transform.translation.x = 0;
      pose_msg.response.transform.transform.translation.y = 0;
      pose_msg.response.transform.transform.translation.z = 0;
    }

    // Transform pose into world space
    geometry_msgs::PoseStamped object_pose, world_pose;
//------
    //JB TODO: arm_navigation_msgs::MoveArmGoal pick_pose = object_goal_map_[object.c_str()].pick_pose;
      geometry_msgs::Pose pick_pose = object_goal_map_[object.c_str()].pick_pose;


    //TODO JB: object_pose.pose.position = pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position;
    //TODO JB: object_pose.pose.orientation = pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;
    object_pose.pose.position = pick_pose.position;
    object_pose.pose.orientation = pick_pose.orientation;
//------

    TransformPoseLocalToWorld(object_pose, world_pose, pose_msg.response.transform);

    state_ = PICKING;
//------
    //TODO JB: pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position = world_pose.pose.position;
    //TODO JB: pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = world_pose.pose.orientation;

//------
    if (stop)
    return;
    /* //TODO JB: if (!SendGoal(pick_pose)) {
      return;
    } */
    if (stop)
    return;
  } else {
    if (stop)
    return;
    //TODO JB: object_goal_map_[object.c_str()].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
    /* //TODO JB: if (!SendGoal(object_goal_map_[object.c_str()].pick_pose)) {
      return;
    } */
    if (stop)
    return;
  }
  r_gripper_.Close();
  state_ = PICKED;
  // Move to Neutral start
  if (stop)
    return;
  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  /* //TODO JB: if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    return;
  } */
  if (stop)
    return;
  state_ = PLACING;
  //TODO JB: object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  /* //TODO JB: if (!SendGoal(object_goal_map_["neutral"].place_pose)) {
    return;
  } */
  if (stop)
    return;

  // obejct place
  //TODO JB: object_goal_map_[object.c_str()].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  /* //TODO JB: if (!SendGoal(object_goal_map_[object.c_str()].place_pose)) {
    return;
  } */
  if (stop)
    return;
  r_gripper_.Open();
  state_ = PLACED;

  if (stop)
    return;
   //TODO JB: object_goal_map_["neutral"].place_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  /* //TODO JB: if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    return;
  } */

  if (stop)
    return;
  //TODO JB: object_goal_map_["neutral"].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  /* //TODO JB: if (!SendGoal(object_goal_map_["neutral"].pick_pose)) {
    return;
  } */
}

bool PickPlace::PickAndPlaceObject(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res) {
  stop = true;
  if (work_thread)
    work_thread->join();
  stop = false;
  work_thread =  boost::shared_ptr<boost::thread>(new boost::thread(&PickAndPlaceThread, this, req.object));
  res.success = true;
  return true;
}

bool PickPlace::PickAndPlacecheck(
    table_setting_demo::pick_and_place::Request &req,
    table_setting_demo::pick_and_place::Response &res) {
  // Check if Object is correct
  // check if Pick and Place is Done
  res.success = (state_ == PLACED);
  if (state_ == PLACED) {
    state_ = IDLE;
  }
  return true;
}

bool PickPlace::PickAndPlaceState(
    table_setting_demo::pick_and_place_state::Request &req,
    table_setting_demo::pick_and_place_state::Response &res) {
  res.state = state_;
  return true;
}

bool PickPlace::PickAndPlaceStop(
    table_setting_demo::pick_and_place_stop::Request &req,
    table_setting_demo::pick_and_place_stop::Response &res) {
  stop = true;
  //TODO JB: move_arm_.cancelGoal();
  work_thread->join();
  stop = false;
  return true;
}

void PickPlace::PostParameters() {
  std::vector<double> positions;
  std::string topic = "/ObjectPositions/";
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) {
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x);
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y);
    //TODO JB: positions.push_back(it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z);
    ros::param::set((topic + it->first).c_str(), positions);
    positions.clear();
  }
}

void waitKeyboard() {
  while (true) {
    int c = getch();
    if (c == ' ')
      break;
    if (c == 10)
      break;
  }
}

bool waitKeyboardYesNo() {
  bool confirm = true;
  while (true) {
    int c = getch();
    if (c == ' ')
      break;
    if (c == 10)
      break;
    if (c == 'n' || c == 'N')
      confirm = false;
  }
  return confirm;
}

void PickPlace::CalibrateObjects() {
  char c;
  r_gripper_.Open();
  bool prior_scene_view = false;
  // Get all dynamic object position with clean view
  printf("Would you like to pre-calibrate dynamic object positions?[Y/n]\n");
  prior_scene_view = waitKeyboardYesNo();
  std::map< std::string, geometry_msgs::Transform> dynamic_transforms;
  if (prior_scene_view) {
    // record scene object positions
    printf("Remove arm from scene!\n");
    waitKeyboard();
    table_setting_demo::object_position pos_msg;
    table_setting_demo::ObjectTransformation pose_msg;

    for (int i = 0; i < dynamic_objects_.size(); ++i) {
      pos_msg.request.object_id = dynamic_objects_[i];
      if (!ros::service::call("qr_get_object_position", pos_msg)) {
        ROS_ERROR("ERROR: Service [%s] no available!", "qr_get_object_position");
      }
      if (pos_msg.response.position.size() > 0) {
        pose_msg.request.x = pos_msg.response.position[0];
        pose_msg.request.y = pos_msg.response.position[1];
        pose_msg.request.w = pos_msg.response.position[2];
        pose_msg.request.h = pos_msg.response.position[3];
        pose_msg.request.object = dynamic_objects_[i];

        if (!ros::service::call("object_transformation", pose_msg)) {
          ROS_ERROR("Service: [%s] not available!", "object_transformation");
        }
      } else {
        pose_msg.response.transform.transform.translation.x = 0;
        pose_msg.response.transform.transform.translation.y = 0;
        pose_msg.response.transform.transform.translation.z = 0;
      }

      // store transformation to a list of dynamic_object transformations
      dynamic_transforms[dynamic_objects_[i]] = pose_msg.response.transform.transform;
    }
  }

  for (uint32_t i = 0; i < objects_.size(); ++i) {
    
    bool dynamic = true;
    for (int j = 0; j < static_objects_.size(); ++j) {
      if (objects_[i] == static_objects_[j]) {
        dynamic = false;
        break;
      }
    }

    if (!dynamic || prior_scene_view) {
      printf(
        "Move [%s] limb to: [%s] picking location and Press Enter\n",
        arm_.c_str(),
        objects_[i].c_str());
      waitKeyboard();
      //TODO JB: object_goal_map_[objects_[i]].pick_pose = GetArmPoseGoal();
    } else {
      printf("Dyanamic Object [%s]! Move arm out of Kinect path! Then Press enter\n", objects_[i].c_str());
      waitKeyboard();
    }
    // check if the object is dynamic
    if (dynamic) {
      table_setting_demo::object_position pos_msg;
      table_setting_demo::ObjectTransformation pose_msg;
      if (!prior_scene_view) {
        // transform into object space
        pos_msg.request.object_id = objects_[i];
        if (!ros::service::call("qr_get_object_position", pos_msg)) {
          ROS_ERROR("Service: [%s] not available!", "qr_get_object_position");
        }
        // Request object 3D transform
        if (pos_msg.response.position.size() > 0) {
          pose_msg.request.x = pos_msg.response.position[0];
          pose_msg.request.y = pos_msg.response.position[1];
          pose_msg.request.w = pos_msg.response.position[2];
          pose_msg.request.h = pos_msg.response.position[3];
          pose_msg.request.object = objects_[i];

          if (!ros::service::call("object_transformation", pose_msg)) {
            ROS_ERROR("Service: [%s] not available!", "object_transformation");
          }
        } else {
          pose_msg.response.transform.transform.translation.x = 0;
          pose_msg.response.transform.transform.translation.y = 0;
          pose_msg.response.transform.transform.translation.z = 0;
        }
      } else {
        pose_msg.response.transform.transform = dynamic_transforms[objects_[i]];
      }
      if (!prior_scene_view) {
        printf(
          "Move [%s] limb to: [%s] picking location and Press Enter\n",
          arm_.c_str(),
          objects_[i].c_str());
        waitKeyboard();
        //TODO JB: object_goal_map_[objects_[i]].pick_pose = GetArmPoseGoal();
      }

      geometry_msgs::PoseStamped world_pose, object_pose;
      //TODO JB: world_pose.pose.position =    object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position;
      //TODO JB: world_pose.pose.orientation = object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;

      // Transform pose
      TransformPoseWorldToLocal(world_pose, object_pose, pose_msg.response.transform);
      // apply transform
      //TODO JB: object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position = object_pose.pose.position;
      //TODO JB: object_goal_map_[objects_[i]].pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = object_pose.pose.orientation;
    }

    r_gripper_.Close();
    printf(
      "Move [%s] limb to: [%s] Placeing location and Press Enter\n",
      arm_.c_str(),
      objects_[i].c_str());
    waitKeyboard();
    //TODO JB: object_goal_map_[objects_[i]].place_pose = GetArmPoseGoal();
    r_gripper_.Open();
  }
}
void PickPlace::ReadCalibration(std::string filename) {
  std::ifstream fin;
  fin.open(filename.c_str(), std::ifstream::binary);
  char header[256];
  char frame_id_str[256],link_str[256]; 
  std::string key;
  Point_t position, orientation;
  // Read header
  fin.read(header, 256);
  std::string frame_id, link;
  sscanf(header,"%s\n%s\n", frame_id_str, link_str);
  frame_id = frame_id_str;
  link = link_str;
  // read in
  while (fin.peek() != EOF) {
    fin.read(header, 128);
    key = header;
    fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
    printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
    fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
    //TODO JB: object_goal_map_[key].pick_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
    fin.read(reinterpret_cast<char*>(&position), sizeof(Point_t));
    printf("Position: x: %f, y: %f, z: %f, \n", position.x, position.y, position.z);
    fin.read(reinterpret_cast<char*>(&orientation), sizeof(Point_t));
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", orientation.x, orientation.y, orientation.z, orientation.w);
    //TODO JB: object_goal_map_[key].place_pose = GetArmPoseFromPoints(frame_id, link, position, orientation);
  }
  fin.close();
}

//-------
/* //TODO JB: arm_navigation_msgs::MoveArmGoal PickPlace::GetArmPoseFromPoints(std::string frame_id, std::string link, Point_t position, Point_t orientation) {
  arm_navigation_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = arm_.c_str();
  goal.motion_plan_request.num_planning_attempts = 5;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh_.param<std::string>(
    "planner_id",
    goal.motion_plan_request.planner_id,
    std::string(""));
  nh_.param<std::string>(
    "planner_service_name",
    goal.planner_service_name,
    std::string("ompl_planning/plan_kinematic_path"));

  // Setup position of Joint
  goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = frame_id.c_str();
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = link.c_str();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = position.x;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = position.y;
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = position.z;

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;

  // Setup Orientation
  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = frame_id.c_str();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = link.c_str();

  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = orientation.x;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = orientation.y;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = orientation.z;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = orientation.w;

  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  return goal;
} */

// geometry_msgs::Pose PickPlace::GetArmPoseFromPoints(std::string frame_id, std::string link, Point_t position, Point_t orientation) {
//   geometry_msgs::Pose goal;

  // goal.motion_plan_request.group_name = arm_.c_str();
  // goal.motion_plan_request.num_planning_attempts = 5;
  // goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  //group.setPlanningTime(5.0);

  // nh_.param<std::string>(
  //   "planner_id",
  //   goal.motion_plan_request.planner_id,
  //   std::string(""));
  // nh_.param<std::string>(
  //   "planner_service_name",
  //   goal.planner_service_name,
  //   std::string("ompl_planning/plan_kinematic_path"));

  // // Setup position of Joint
  // goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = frame_id.c_str();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = link.c_str();
  // goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = position.x;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = position.y;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = position.z;

  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  // goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  // goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;

  // // Setup Orientation
  // goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = frame_id.c_str();
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = link.c_str();

  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = orientation.x;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = orientation.y;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = orientation.z;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = orientation.w;

  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  // goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  // return goal;
// } 

//-------


void PickPlace::SaveCalibration(std::string filename) {
  std::ofstream fout;
  fout.open(filename.c_str(), std::ofstream::binary);
  char header[256];
  // Save links to first 
  snprintf(header, 256, "%s\n%s\n", "torso_lift_link", "r_wrist_roll_link");
  fout.write(header, 256);
  // save positions
  Point_t point;
  point.w = 1;
  for (std::map<std::string, PickPlaceGoal>::iterator it = object_goal_map_.begin();
      it != object_goal_map_.end();
      ++it) {
    snprintf(header, 128, "%s", it->first.c_str());
    fout.write(header, 128);
    printf("Key:%s\n", header);
    /* //TODO JB: point.x = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x;
    point.y = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y;
    point.z = it->second.pick_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z;
    printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    point.x = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x;
    point.y = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y;
    point.z = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z;
    point.w = it->second.pick_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w;
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    */
    /* //TODO JB: point.x = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.x;
    point.y = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.y;
    point.z = it->second.place_pose.motion_plan_request.goal_constraints.position_constraints[0].position.z;
    printf("Position: x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    point.x = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x;
    point.y = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y;
    point.z = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z;
    point.w = it->second.place_pose.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w;
    printf("Orientation: x: %f, y: %f, z: %f, w: %f\n", point.x, point.y, point.z, point.w);
    fout.write(reinterpret_cast<char*>(&point), sizeof(point));
    */
  }
}

/* //TODO JB: bool PickPlace::SendGoal(arm_navigation_msgs::MoveArmGoal goal) {
  if (nh_.ok()) {
    bool finished_within_time = false;
    ROS_INFO("Sending Goal");
    move_arm_.sendGoal(goal);
    finished_within_time = move_arm_.waitForResult(ros::Duration(45.0));
    if (!finished_within_time) {
      move_arm_.cancelGoal();
      ROS_INFO("Timed out achieving Goal");
    } else {
      actionlib::SimpleClientGoalState state = move_arm_.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if (success) {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
      } else {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        return false;
      }
    }
  }
  return false;
} */

bool PickPlace::SendGoal(geometry_msgs::Pose goal) {
  if (nh_.ok()) {
    bool finished_within_time = false;
    ROS_INFO("Sending Goal");
    // move_arm_.sendGoal(goal);
    // finished_within_time = move_arm_.waitForResult(ros::Duration(45.0));
    group.setPoseTarget(goal);

    // group.move();
    // if (!finished_within_time) {
    //   move_arm_.cancelGoal();
    //   ROS_INFO("Timed out achieving Goal");
    // } else {
      // actionlib::SimpleClientGoalState state = move_arm_.getState();
      // bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      // if (success) {
      //   ROS_INFO("Action finished: %s",state.toString().c_str());
      //   return true;
      // } else {
      //   ROS_INFO("Action failed: %s",state.toString().c_str());
      //   return false;
      // }
    // }
  }
  return false;
} 


/* //TODO JB: arm_navigation_msgs::MoveArmGoal PickPlace::GetArmPoseGoal() {
  arm_navigation_msgs::MoveArmGoal goal;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  // Get Current Arm Pose
  listener.waitForTransform("torso_lift_link", "r_wrist_roll_link",
    ros::Time(0), ros::Duration(3.0));
  listener.lookupTransform("torso_lift_link", "r_wrist_roll_link",
    ros::Time(0), transform);
  goal.motion_plan_request.group_name = arm_.c_str();
  goal.motion_plan_request.num_planning_attempts = 5;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh_.param<std::string>(
    "planner_id",
    goal.motion_plan_request.planner_id,
    std::string(""));
  nh_.param<std::string>(
    "planner_service_name",
    goal.planner_service_name,
    std::string("ompl_planning/plan_kinematic_path"));

  // Setup position of Joint
  goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = transform.getOrigin().x();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = transform.getOrigin().y();
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = transform.getOrigin().z();

  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 0.8;

  // Setup Orientation
  goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = transform.getRotation().x();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = transform.getRotation().y();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = transform.getRotation().z();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = transform.getRotation().w();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.08;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 0.8;

  return goal;
} */
}  // namespace pr2
