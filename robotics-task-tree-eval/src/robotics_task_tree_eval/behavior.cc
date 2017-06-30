/*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "robotics_task_tree_eval/behavior.h"
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include "remote_mutex/remote_mutex.h"
#include "geometry_msgs/Pose.h"

namespace task_net {
typedef std::vector<NodeId_t>::iterator NodeId_t_iterator;
// BEHAVIOR
Behavior::Behavior() {}
Behavior::Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Node(name,
      peers,
      children,
      parent,
      state) {  
      // printf("Behavior::Behavior WAS CALLED\n");
}
Behavior::~Behavior() {}

////////////////////////////////////////////////////////////////////////////////
// AND BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
AndBehavior::AndBehavior() {}
AndBehavior::AndBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
        // printf("AndBehavior::AndBehavior WAS CALLED\n");
    }
AndBehavior::~AndBehavior() {}

void AndBehavior::UpdateActivationPotential() {
    // ROS_INFO("AndBehavior::UpdateActivationPotential was called!!!!\n");

  float sum = 0;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    sum += (*it)->state.activation_potential;
  }
  state_.activation_potential = sum / children_.size();
}

bool AndBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t AndBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = 1.0f / children_.size();
  msg->done = false;

  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    SendToChild((*it)->mask, msg);
  }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// THEN BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
ThenBehavior::ThenBehavior() {}
ThenBehavior::ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  // Initialize activation queue
      // printf("ThenBehavior::ThenBehavior WAS CALLED\n");
  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    activation_queue_.push(*it);
  }
}
ThenBehavior::~ThenBehavior() {}

void ThenBehavior::UpdateActivationPotential() {
    // ROS_INFO("ThenBehavior::UpdateActivationPotential was called!!!!\n");
  float sum = 0;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    sum += (*it)->state.activation_potential;
  }
  state_.activation_potential = sum / children_.size();
}

bool ThenBehavior::Precondition() {
    // ROS_INFO("ThenBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t ThenBehavior::SpreadActivation() {
    // ROS_INFO("ThenBehavior::SpreadActivation was called!!!!");
  if (!activation_queue_.empty()) {
    ControlMessagePtr_t msg(new ControlMessage_t);
    msg->sender = mask_;
    msg->activation_level = 1.0f;
    msg->done = false;

    if (activation_queue_.front()->state.done) {
      activation_queue_.pop();
    }

    SendToChild(activation_queue_.front()->mask, msg);
  }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// OR BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OrBehavior::OrBehavior() {}
OrBehavior::OrBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  seed = static_cast<uint32_t>(time(NULL));
  random_child_selection = rand_r(&seed) % children_.size();
  // printf("OrBehavior::OrBehavior WAS CALLED\n");
}
OrBehavior::~OrBehavior() {}

void OrBehavior::UpdateActivationPotential() {
    // ROS_INFO("OrBehavior::UpdateActivationPotential was called!!!!\n");
  float max = 0;
  int max_child_index = 0, index = 0;

  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    float value = (*it)->state.activation_potential;
    if (value > max) {
      max = value;
      max_child_index = index;
    }
    index++;
  }
  state_.activation_potential = max;
  random_child_selection = max_child_index;
}

bool OrBehavior::Precondition() {
    // ROS_INFO("OrBehavior::Precondition was called!!!!\n");
  if (children_[random_child_selection]->state.done)
    return true;
  return false;
}

uint32_t OrBehavior::SpreadActivation() {
    // ROS_INFO("OrBehavior::SpreadActivation was called!!!!");
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = 1.0f;
  msg->done = false;

  SendToChild(children_[random_child_selection]->mask, msg);
}


////////////////////////////////////////////////////////////////////////////////
// DUMMY PLACE BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DummyBehavior::DummyBehavior() {}
DummyBehavior::DummyBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    ROBOT robot_des,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state), mut_arm(name.topic.c_str(), "/right_arm_mutex") {
        // printf("DummyBehavior::DummyBehavior WAS CALLED\n
    object_ = object;
    robot_des_ = robot_des;
}
DummyBehavior::~DummyBehavior() {}

void DummyBehavior::UpdateActivationPotential() {
    // ROS_INFO("AndBehavior::UpdateActivationPotential was called!!!!\n");

  state_.activation_potential = 100;
}

bool DummyBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  return true;
}

uint32_t DummyBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  // ControlMessagePtr_t msg(new ControlMessage_t);
  // msg->sender = mask_;
  // msg->activation_level = 1.0f;
  // msg->done = false;
}

void DummyBehavior::Work() {
  ROS_INFO("DummyBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));

  PickAndPlace(object_, robot_des_);
    // while (!PickAndPlaceDone()) {
      boost::this_thread::sleep(boost::posix_time::millisec(500));
        // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
    // }
  mut_arm.Release();
  ROS_INFO("DummyBehavior::Work: Done!");
  ROS_INFO("\tDmmyBehavior::MUTEX IS RELEASED!");
}

void DummyBehavior::PickAndPlace(std::string object, ROBOT robot_des) {

  // pick
  table_task_sim::PickUpObject req_pick;
  req_pick.request.robot_id = (int)robot_des;
  req_pick.request.object_name = object;
  // table_task_sim::PickUpObject::Response res_pick; //to know if it failed or not...

  // place
  geometry_msgs::Pose pose;
  pose.position.x = -0.25;
  pose.position.y = -.025;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  table_task_sim::PlaceObject req_place;
  req_place.request.robot_id = (int)robot_des;
  req_place.request.goal = pose;
  // table_task_sim::PlaceObject::Response res_place; //to know if it failed or not...

  // call the pick service
  if(ros::service::call("pick_service", req_pick)) {

    ROS_INFO("\n\n\n\t\t THE PICK SERVICE WAS CALLED!!");

    // call the place service
    if(ros::service::call("place_service", req_place)) {
      ROS_INFO("\n\n\t\t THE PLACE SERVICE WAS CALLED!!\n\n\n");
    }
  }

}

bool DummyBehavior::PickAndPlaceDone() {
  // table_setting_demo::pick_and_place msg;
  // msg.request.object = object_;
  // ros::service::call("pick_and_place_check", msg);
  // return msg.response.success;
}

bool DummyBehavior::ActivationPrecondition() {
  ROS_INFO("\tDmmyBehavior::MUTEX IS LOCKING!");

  return mut_arm.Lock(state_.activation_potential);
  // return true;
}

}  // namespace task_net
