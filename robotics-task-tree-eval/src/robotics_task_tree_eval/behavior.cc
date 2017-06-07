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
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

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
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
        // printf("DummyBehavior::DummyBehavior WAS CALLED\n");
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
  boost::this_thread::sleep(boost::posix_time::millisec(10000));
  mut.Release();
  ROS_INFO("DummyBehavior::Work: Done!");
}

bool DummyBehavior::ActivationPrecondition() {
  return mut.Lock(state_.activation_potential);
  return true;
}

}  // namespace task_net
