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
#include "robotics_task_tree_eval/node.h"
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <stdlib.h>
#include <string>
#include <vector>
#include "robotics_task_tree_eval/State.h"
#include "log.h"

namespace task_net {

#define PUB_SUB_QUEUE_SIZE 100
#define STATE_MSG_LEN (sizeof(State))
#define ACTIVATION_THESH 0.1
#define ACTIVATION_FALLOFF 0.95f

void PeerCheckThread(Node *node);


Node::Node() {
  state_.active = false;
  state_.done = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    bool use_local_callback_queue, boost::posix_time::millisec mtime):
    local_("~") {
  if (use_local_callback_queue) {
    LOG_INFO("Local Callback Queues");
    pub_nh_.setCallbackQueue(pub_callback_queue_);
    sub_nh_.setCallbackQueue(sub_callback_queue_);
  }
    // ROS_INFO("Node::Node was called!!!!\n");

  // Generate reverse map
  GenerateNodeBitmaskMap();
  name_   = node_dict_[GetBitmask(name.topic)];

  for (NodeListIterator it = peers.begin(); it != peers.end(); ++it) {
    if(strcmp(it->topic.c_str(), "NONE") != 0) {
      peers_.push_back(node_dict_[GetBitmask(it->topic)]);
      // NOTE: THIS IS PROBABLY IN THE WRONG SPOT NOW BUT IT WAS CAUSING ISSUES BELOW!
    }
  }

  for (NodeListIterator it = children.begin(); it != children.end(); ++it) {
    children_.push_back(node_dict_[GetBitmask(it->topic)]);
  }
  parent_ = node_dict_[GetBitmask(parent.topic)];
  // Setup bitmasks
  InitializeBitmask(name_);
  InitializeBitmasks(peers_);
  InitializeBitmasks(children_);
  InitializeBitmask(parent_);

  state_ = state;
  state_.owner = name_->mask;
  state_.active = false;
  state_.done = false;
  state_.activation_level = 0.0f;
  state_.activation_potential = 0.0f;
  state_.peer_active = false;
  state_.peer_done = false;
  state_.check_peer = false;
  state_.peer_okay = false;

  // Get bitmask
  // printf("name: %s\n", name_->topic.c_str());
  mask_ = GetBitmask(name_->topic);

  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_, "_parent");
  InitializePublishers(peers_, &peer_pub_list_, "_peer");
  InitializePublisher(parent_, &parent_pub_);
  InitializeStatePublisher(name_, &self_pub_, "_state");

  NodeInit(mtime);

}

Node::~Node() {}

void Node::InitializeBitmask(NodeId_t * node) {
    // ROS_INFO("Node::InitializeBitmask was called!!!!\n");
  node->mask = GetBitmask(node->topic);
}

void Node::InitializeBitmasks(NodeListPtr nodes) {
    // ROS_INFO("Node::InitializeBitmasks was called!!!!\n");
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    InitializeBitmask(*it);
  }
}

void Node::GenerateNodeBitmaskMap() {
      // ROS_INFO("Node::GenerateNodeBitmaskMap was called!!!!\n");

  std::vector<std::string> nodes;
  if (local_.getParam("NodeList", nodes)) {
    // printf("Generating BitmaskMap\n");
    for (std::vector<std::string>::iterator it = nodes.begin();
        it != nodes.end(); ++it) {
      NodeId_t *nptr = new NodeId_t;
      nptr->topic = *it;
      nptr->mask = GetBitmask(*it);
      nptr->pub = NULL;
      nptr->state =  {nptr->mask, false, false, 0.0f, 0.0f};
      node_dict_[nptr->mask] = nptr;
      // printf("Adding [%s] to Dictionary.\n", nptr->topic.c_str());
    }
  }
}
void Node::Activate() {
    // ROS_INFO("Node::Activate was called!!!!\n");

  state_.check_peer = true;
  // TODO JB: have this only spin a new thread if the thread doesn't already exist 
  // create peer_check thread if it isn't already running 
  if(!peer_check_thread) {
    peer_check_thread  = new boost::thread(&PeerCheckThread, this); 
    printf("\n\tThread was not active, so has been created!\n");
    peer_check_thread->detach(); 
  }
  else {
        printf("\n\tThread was already active\n");
      }

 // if thread is okay, run this??
 if(state_.peer_okay) {
      ROS_INFO("NODE::Activate: peer has made it into the if statement!!!");
    if (!state_.active && !state_.done) {
      if (ActivationPrecondition()) {
        ROS_INFO("Activating Node: %s", name_->topic.c_str());
        // printf("\t\tNode::Activate Activating Node: %s\n\n", name_->topic.c_str());
        {
          boost::lock_guard<boost::mutex> lock(work_mut);
          state_.active = true;
          ROS_INFO("State was set to true!");
          // Send activation to peers to avoid race condition
          // this will publish the updated state to say I am now active
          PublishStateToPeers(); 
        }
        cv.notify_all();
        // TODO JB: kill the thread now
        peer_check_thread->interrupt();
        peer_check_thread = NULL;
        }
    }
    state_.check_peer = false;
    state_.peer_okay = false;
    ROS_INFO("NODE::ACTIVATE: check peer set back to false!!!");
    
  }
}

bool Node::ActivationPrecondition() {
    // ROS_INFO("Node::ActivationPrecondition was called!!!!\n");
  return true;
}

void Node::Deactivate() {
    // ROS_INFO("Node::Deactivate was called!!!!\n");
  // if (state_.active && state_.owner == name_.c_str()) {
  //   state_.active = false;
  //   printf("Deactivating Node; %s\n", name_.c_str());
  // }
}

void Node::ActivateNode(NodeId_t node) {    
  // ROS_INFO("Node::ActivateNode was called!!!!\n");
}

void Node::DeactivateNode(NodeId_t node) {
      // ROS_INFO("Node::DeactivateNode was called!!!!\n");
}

void Node::Finish() {
      ROS_INFO("Node::Finish was called!!!!\n");

  Deactivate();
  state_.done = true;
}

State Node::GetState() {
      // ROS_INFO("Node::GetState was called!!!!\n");

  return state_;
}

void Node::SendToParent(const robotics_task_tree_eval::ControlMessage msg) {
    // ROS_INFO("Node::SendToParent was called!!!!\n");
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  parent_pub_.publish(msg_temp);
}
void Node::SendToParent(const ControlMessagePtr_t msg) {
    // ROS_INFO("Node::SendToParent was called!!!!\n");
  parent_pub_.publish(msg);
}
void Node::SendToChild(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
    // ROS_INFO("Node::SendToChild was called!!!!\n");
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToChild(NodeBitmask node, const ControlMessagePtr_t msg) {
    // ROS_INFO("Node::SendToChild was called!!!!\n");
  node_dict_[node]->pub->publish(msg);
}
void Node::SendToPeer(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
    // ROS_INFO("Node::SendToPeer was called!!!!\n");
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);

}
void Node::SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg) {
    // ROS_INFO("Node::SendToPeer was called!!!!\n");
  node_dict_[node]->pub->publish(msg);
}

void Node::ReceiveFromParent(ConstControlMessagePtr_t msg) {
    // ROS_INFO("Node::ReceiveFromParent was called!!!!\n");
  // Set activation level from parent
  // TODO(Luke Fraser) Use mutex to avoid race condition setup in publisher
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level = msg->activation_level;
}
void Node::ReceiveFromChildren(ConstControlMessagePtr_t msg) {
    // ROS_INFO("Node::ReceiveFromChildren was called!!!!\n");
  // Determine the child
  NodeId_t *child = node_dict_[msg->sender];
  boost::unique_lock<boost::mutex> lck(mut);
  child->state.activation_level = msg->activation_level;
  child->state.activation_potential = msg->activation_potential;
  child->state.done = msg->done;
  // ROS_INFO("child->state.activation_level %f", child->state.activation_level);
  // ROS_INFO("child->state.activation_potential %f", child->state.activation_potential);
  child->state.active = msg->active;
}
void Node::ReceiveFromPeers(ConstControlMessagePtr_t msg) {
    // ROS_INFO("Node::ReceiveFromPeers was called!!!!\n");
  // boost::unique_lock<boost::mutex> lck(mut);
  // state_.activation_level = msg->activation_level;
  // state_.done = msg->done;
  boost::unique_lock<boost::mutex> lck(mut);
  state_.peer_active = msg->active;
  state_.peer_done = msg->done;
  state_.done = state_.done || state_.peer_done;
}

// Main Loop of Update Thread. spins once every mtime milliseconds
void UpdateThread(Node *node, boost::posix_time::millisec mtime) {
    ROS_INFO("Node::UpdateThread was called!!!!\n");
  while (true) {
    node->Update();
    boost::this_thread::sleep(mtime);
  }
}

// TODO: need to be able to reset node if work fails
// TODO: Need to be able to cancel work as well.
// IDEA: a work master is started to hault work if necessary.
// IDEA: This thread may be able to start the thread then become the work watcher
// IDEA: The work watcher may need to funtion earlier than the work thread is started.
void WorkThread(Node *node) {
      ROS_INFO("Node::WorkThread was called!!!!\n");

  boost::unique_lock<boost::mutex> lock(node->work_mut);
  while (!node->state_.active) {
    node->cv.wait(lock);
  }
  LOG_INFO("work thread Initialized");
  // Process Data
  node->working = true;
  node->Work();
  boost::unique_lock<boost::mutex> lck(node->mut);
  node->state_.active = false;
  node->state_.done = true;
  node->working = false;
  node->PublishDoneParent();
  node->PublishStateToPeers();
}

// TODO JB: implementation for peer thread!
void PeerCheckThread(Node *node) {
      // ROS_INFO("Node::PeerCheckThread was called!!!!\n");

  // wait for checking to be asked!
  boost::unique_lock<boost::mutex> lockp(node->peer_mut);
   while (!node->state_.check_peer) {
    ROS_INFO("PeerCheckThread is waiting!");
    node->cv.wait(lockp);
  }
  // LOG_INFO("check peer thread Initialized");
  // notify peers I want to start this node 
  // by sending status and activation potential to peers
  node->PublishStateToPeers(); 

  // TODO: In the futurre maybe make a recieve from peers call here to ensure
  // that this happens right since the timing of the return from the check 
  // causing issues for THEN without some hard-coded offset as below?!?!

  // wait for full loop so can recieved data back from peers
  // NOTE: Due to the exact same timing in the THEN case, change the loop time to deal
  //       with latency for the different sets of nodes
  int buff = node->state_.owner.robot;
  printf("\n\n\n\t\t\tBUFF: %d \tTOTAL TIME: %d\n\n\n", buff, 500+(buff*1500));
  boost::this_thread::sleep(boost::posix_time::millisec(500+(buff*1500)));  

  // for each peer, check status
  // (might have to change logic to take highest of all peers?!?)
  // for now just assume only 1 peer!!!
  for (NodeListPtr::iterator it = node->peers_.begin();
      it != node->peers_.end(); ++it) {

    // printf("\n\nPeer DATA:\t%s\n\tactive: %d\tdone:%d\n\n", (*it)->topic.c_str(),(*it)->state.active,(*it)->state.done);
    printf("\n\nPeer DATA:\t%s\n\tactive: %d\tdone:%d\n\n", (*it)->topic.c_str(),node->state_.peer_active,node->state_.peer_done);
    printf("\n\nMe   DATA:\t%s\n\tactive: %d\tdone:%d\n\n", node->name_->topic.c_str(),node->state_.active,node->state_.done);


    // if peer done, then peer_okay = False (since already completed, I can't activate) 
    // if((*it)->state.done) {
    if(node->state_.peer_done) {
       printf("\n\nPeerCheckThread: Case 1!!\n\n");
      node->state_.peer_okay = false; 
    }
    // otherwise if peer active
    // else if ((*it)->state.active) {
    else if (node->state_.peer_active) {
      // if ((*it)->state.activation_potential < node->state_.activation_potential) {
      //   // if my activation potential/level (which one? potential right)
      //   // is > my peer's activation potentional/level, then peer_okay = True
      //   // NOTE: I don't think this will ever happen, becuase I am not active yet so if
      //   // my peer is already active, then it made it through this process and so it wont
      //   // be stopped from doing work already?!?!
      //  printf("\n\nPeerCheckThread: Case 2!!\n\n");
      //  node->state_.peer_okay = true; 
      // }
      //   // otherwise mine < peer, so let peer be set to active, implies peer_okay = False 
      // else{ 
       printf("\n\nPeerCheckThread: Case 3!!\n");
      node->state_.peer_okay = false; 
      // lower my activation level for this node
      printf("\tCurr level: %f\n", node->state_.activation_level);
      node->state_.activation_level = ACTIVATION_FALLOFF*node->state_.activation_level;
      printf("\tNew level: %f\n\n", node->state_.activation_level);
      // }

    }
    // otherwise, peer is not active and peer is not done so I can activate, peer_okay = True
    else if (!node->state_.peer_done && !node->state_.peer_active) {
       printf("\n\nPeerCheckThread: Case 4!!\n\n");
      node->state_.peer_okay = true; 
    }
    else {
      printf("\n\nERROR! PeerCheckThread: Undefined case! Please redo logic!\n\n");
    }
  }
  printf("\nPeercheckthread is at end!!!!\n");
}

void CheckThread(Node *node) {
      // ROS_INFO("Node::CheckThread was called!!!!\n");

  boost::mutex mut;
  boost::unique_lock<boost::mutex> lock(mut);
  while (!node->state_.active) {
    LOG_INFO("Check Thread waiting");
    // ROS_INFO("Check Thread waiting");
    node->cv.wait(lock);
  }
  LOG_INFO("Check Work Thread Initialized");
  // ROS_INFO("Check Work Thread Initialized");
  while (node->state_.active) {
    if (!node->CheckWork()) {
      LOG_INFO("Deleting Thread! and Restarting");
      // ROS_INFO("Deleting Thread! and Restarting");
      {
        boost::unique_lock<boost::mutex> lock(node->mut);
        node->work_thread->interrupt();
        delete node->work_thread;
        node->UndoWork();
        node->working = false;
        node->state_.active = false;
        node->work_thread = new boost::thread(&WorkThread, node);
        node->check_thread = new boost::thread(&CheckThread, node);
        break;
      }
    }
    boost::this_thread::sleep(boost::posix_time::millisec(10));
  }
}

void Node::RecordToFile() {
      // ROS_INFO("Node::RecordToFile was called!!!!\n");

  boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
  boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - time_t_epoch;
  double seconds = (double)diff.total_seconds() + (double)diff.fractional_seconds() / 1000000.0;
  record_file  << std::fixed
        << seconds
        << ", "
        << state_.active
        << ", "
        << state_.done
        << ", "
        << state_.activation_level
        << ", "
        << state_.activation_potential
        << ","
        << working
        << "\n";
        record_file.flush();
}
void RecordThread(Node *node) {
      // ROS_INFO("Node::RecordThreadc was called!!!!\n");

  // Open Record File
  while (true) {
    node->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }
}
// Initialize node threads and variables
void Node::NodeInit(boost::posix_time::millisec mtime) {
  // ROS_INFO("Node::NodeInit was called!!!!\n");

  // Initialize node threads
  update_thread = new boost::thread(&UpdateThread, this, mtime);
  work_thread   = new boost::thread(&WorkThread, this);
  check_thread  = new boost::thread(&CheckThread, this);
  // peer_check_thread  = new boost::thread(&PeerCheckThread, this);

  // Initialize recording Thread
  std::string filename = "~/catkin_ws/src/Distributed_Collaborative_Task_Tree/Data/" + name_->topic + "_Data_.csv";
  ROS_INFO("Creating Data File: %s", filename.c_str());
  record_file.open(filename.c_str());
  record_file.precision(15);
  record_thread = new boost::thread(&RecordThread, this);
}

void Node::ActivationFalloff() {
    // ROS_INFO("Node::ActivationFalloff was called!!!!\n");
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level *= ACTIVATION_FALLOFF;
}
// Main Loop of the Node type Each Node Will have this fucnction called at each
// times step to process node properties. Each node should run in its own thread
void Node::Update() {
      // ROS_INFO("Node::Update was called!!!!\n");

  // Check if Done
  if (!IsDone()) {
    // Check Activation Level
    if (IsActive()) {
      // Check Preconditions
      if (Precondition()) {
        ROS_INFO("Node: %s - Preconditions Satisfied Safe To Do Work!",
          name_->topic.c_str());
        Activate();
      } else {
          ROS_INFO("Node: %s|Preconditions Not Satisfied, Spreading Activation!",
            name_->topic.c_str());
        SpreadActivation();
      }
      ActivationFalloff();
    }
    else {
      ROS_INFO("Node: %s - Not Active: %f", name_->topic.c_str(),
        state_.activation_level); }
  }
  // Publish Status
  PublishStatus();
}

void Node::Work() {
    // ROS_INFO("Node::Work was called!!!!\n");
  printf("Doing Work\n");
  boost::this_thread::sleep(boost::posix_time::millisec(1000));
  printf("Done!\n");
}

bool Node::CheckWork() {
    // ROS_INFO("Node::CheckWork was called!!!!\n");
  // LOG_INFO("Checking Work");
  boost::this_thread::sleep(boost::posix_time::millisec(100));
  return true;
}

void Node::UndoWork() {
      // ROS_INFO("Node::UndoWork was called!!!!\n");

  LOG_INFO("Undoing Work");
}
// Deprecated function. use ros message data type with struct generality.
std::string StateToString(State state) {
    // ROS_INFO("StateToString was called!!!!\n");

  char buffer[sizeof(State)*8];
  snprintf(buffer, sizeof(buffer), "Owner:%u, Active:%d, Done:%d, Level:%f",
    *reinterpret_cast<uint32_t*>(&state),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)+sizeof(bool)),
    *(reinterpret_cast<float*>(&state)+sizeof(NodeBitmask)+sizeof(bool)*2));
  std::string str = buffer;
  return str;
}

void Node::PublishStatus() {
    // ROS_INFO("Node::PublishStatus was called!!!!\n");
  boost::shared_ptr<State_t> msg(new State_t);
  *msg = state_;
  self_pub_.publish(msg);

  // Publish Activation Potential
  PublishActivationPotential();
  PublishStateToPeers();
}

void Node::PublishStateToPeers() {
    // ROS_INFO("Node::PublishStateToPeers was called!!!!\n");
  boost::shared_ptr<ControlMessage_t> msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;

  for (PubList::iterator it = peer_pub_list_.begin();
      it != peer_pub_list_.end(); ++it) {
    it->publish(msg);
  }
}

void Node::PublishActivationPotential() {
    // ROS_INFO("Node::PublishActivationPotential was called!!!!\n");
  // Update Activation Potential
  UpdateActivationPotential();
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  // ROS_INFO("msg->activation_level %f", msg->activation_level);
  // ROS_INFO("msg->activation_potential %f", msg->activation_potential);
  msg->active = state_.active;
  parent_pub_.publish(msg);
}

void Node::UpdateActivationPotential() {
      // ROS_INFO("Node::UpdateActivationPotential was called!!!!\n");

}

void Node::PublishDoneParent() {
      // ROS_INFO("Node::PublishDoneParent was called!!!!\n");

  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = state_.activation_level;
  msg->activation_potential = state_.activation_potential;
  msg->done = state_.done;
  msg->active = state_.active;
  parent_pub_.publish(msg);
  // printf("Publish Status: %d\n", msg->done);
}

bool Node::IsDone() {
      // ROS_INFO("Node::IsDone was called!!!!\n");

  return state_.done;
}
bool Node::IsActive() {
      // ROS_INFO("Node::IsActive was called!!!!\n");

  return state_.activation_level > ACTIVATION_THESH;
}
float Node::ActivationLevel() {
      // ROS_INFO("Node::ActivationLevel was called!!!!\n");

  return state_.activation_level;
}
bool Node::Precondition() {
      // ROS_INFO("Node::Precondition was called!!!!\n");

  // TODO(Luke Fraser) Merge children/peer/name/parent lists to point to the
  // same as dictionary
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}
uint32_t Node::SpreadActivation() {
      // ROS_INFO("Node::SpreadActivation was called!!!!\n");

}
void Node::InitializeSubscriber(NodeId_t *node) {
    // ROS_INFO("Node::InitializeSubscriber was called!!!!\n");
  std::string peer_topic = node->topic + "_peer";
#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Peer Topic: %s\n", peer_topic.c_str());
#endif
  peer_sub_     = sub_nh_.subscribe(peer_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromPeers,
    this);

#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Child Topic: %s\n", node->topic.c_str());
#endif
  children_sub_ = sub_nh_.subscribe(node->topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromChildren,
    this);
  std::string parent_topic = node->topic + "_parent";
#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Parent Topic: %s\n", parent_topic.c_str());
#endif
  parent_sub_ = sub_nh_.subscribe(parent_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromParent,
    this);
}
void Node::InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition) {
    // ROS_INFO("Node::InitializePublishers was called!!!!\n");
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    ros::Publisher * topic = new ros::Publisher;
    *topic =
      pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(
        (*it)->topic + topic_addition,
        PUB_SUB_QUEUE_SIZE);

    pub->push_back(*topic);
    node_dict_[(*it)->mask]->pub = topic;
    node_dict_[(*it)->mask]->topic += topic_addition;
#if DEBUG
    printf("[PUBLISHER] - Creating Topic: %s\n", (*it)->topic.c_str());
#endif
  }
}

void Node::InitializePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition) {
      // ROS_INFO("Node::InitializePublisher was called!!!!\n");

  node->topic += topic_addition;
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", node->topic.c_str());
#endif
  (*pub) =
    pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(node->topic,
      PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

void Node::InitializeStatePublisher(NodeId_t *node, ros::Publisher *pub,
  const char * topic_addition) {
      // ROS_INFO("Node::InitializeStatePublisher was called!!!!\n");

  node->topic += topic_addition;
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", node->topic.c_str());
#endif
  (*pub) = pub_nh_.advertise<robotics_task_tree_eval::State>(node->topic,
    PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

NodeBitmask Node::GetBitmask(std::string name) {
    // ROS_INFO("Node::GetBitmask was called!!!!\n");
  // Split underscores
  std::vector<std::string> split_vec;
  boost::algorithm::split(split_vec, name,
    boost::algorithm::is_any_of("_"));
  NodeBitmask mask;
  // node_type
  mask.type  = static_cast<uint8_t>(atoi(split_vec[1].c_str()));
  mask.robot = static_cast<uint8_t>(atoi(split_vec[2].c_str()));
  mask.node  = static_cast<uint16_t>(atoi(split_vec[3].c_str()));
  return mask;
}
NodeId_t Node::GetNodeId(NodeBitmask id) {
    // ROS_INFO("Node::GetNodeId was called!!!!\n");
  return *node_dict_[id];
}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
    // ROS_INFO("Node::GetPubCallbackQueue was called!!!!\n");
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
    // ROS_INFO("Node::GetSubCallbackQueue was called!!!!\n");
  return sub_callback_queue_;
}
}  // namespace task_net
