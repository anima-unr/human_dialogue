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
#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdint.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include "robotics_task_tree_eval/node_types.h"
#include "robotics_task_tree_eval/ControlMessage.h"

namespace task_net {

typedef boost::shared_ptr<robotics_task_tree_eval::ControlMessage const>
  ConstControlMessagePtr;
typedef boost::shared_ptr<robotics_task_tree_eval::ControlMessage>
  ControlMessagePtr;
typedef boost::shared_ptr<ControlMessage_t const>
  ConstControlMessagePtr_t;
typedef boost::shared_ptr<ControlMessage_t>
  ControlMessagePtr_t;

// Pre-declare
class Node;
void WorkThread(Node* node);
/*
Class: Node
Definition: Base class for behavior network nodes. All nodes will inherit from
            this class. This list includes AND, THEN, OR, WHILE nodes.
Author: Luke Fraser
*/
class Node {
 public:
  Node();
  Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000));
  virtual ~Node();

  virtual void Update();
  virtual void Work();
  virtual bool CheckWork();
  virtual void UndoWork();

 protected:
  virtual void Activate();
  virtual void Deactivate();
  virtual void ActivateNode(NodeId_t node);
  virtual void DeactivateNode(NodeId_t node);
  virtual void Finish();
  virtual State GetState();

  // Messaging
  virtual void SendToParent(
    const robotics_task_tree_eval::ControlMessage msg);
  virtual void SendToParent(const ControlMessagePtr_t msg);
  virtual void SendToChild(NodeBitmask node,
    const robotics_task_tree_eval::ControlMessage msg);
  virtual void SendToChild(NodeBitmask node, const ControlMessagePtr_t msg);
  virtual void SendToPeer(NodeBitmask node,
    const robotics_task_tree_eval::ControlMessage msg);
  virtual void SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg);

  // Receiving Threads
  virtual void ReceiveFromParent(ConstControlMessagePtr_t msg);
  virtual void ReceiveFromChildren(ConstControlMessagePtr_t msg);
  virtual void ReceiveFromPeers(ConstControlMessagePtr_t msg);

  // Main Node loop functions
  virtual bool IsDone();
  virtual bool IsActive();
  virtual float ActivationLevel();
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
  virtual ros::CallbackQueue* GetPubCallbackQueue();
  virtual ros::CallbackQueue* GetSubCallbackQueue();

  ros::CallbackQueue* pub_callback_queue_;
  ros::CallbackQueue* sub_callback_queue_;


  friend void WorkThread(Node* node);
  friend void RecordThread(Node* node);
  friend void CheckThread(Node* node);

  virtual void RecordToFile();
 private:
  virtual void NodeInit(boost::posix_time::millisec mtime);
  virtual void PublishStatus();
  virtual void PublishActivationPotential();
  virtual void UpdateActivationPotential();
  virtual void PublishDoneParent();
  virtual void InitializeSubscriber(NodeId_t *node);
  virtual void InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition = "");
  virtual void InitializePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition = "");
  virtual void InitializeStatePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition = "");
  virtual NodeBitmask GetBitmask(std::string name);
  virtual NodeId_t GetNodeId(NodeBitmask id);
  virtual void GenerateNodeBitmaskMap();
  virtual void InitializeBitmask(NodeId_t* node);
  virtual void InitializeBitmasks(NodeListPtr nodes);
  virtual bool ActivationPrecondition();
  virtual void ActivationFalloff();
  virtual void PublishStateToPeers();

 protected:
  std::ofstream record_file;
  NodeId_t *name_;
  State state_;
  std::map<NodeBitmask, NodeId_t*, BitmaskLessThan> node_dict_;
  std::string name_id_;
  NodeBitmask mask_;
  NodeListPtr peers_;
  NodeListPtr children_;
  NodeId_t *parent_;

  // Publishers
  PubList children_pub_list_;
  PubList peer_pub_list_;
  ros::Publisher parent_pub_;
  ros::Publisher self_pub_;

  // Subscribers
  ros::Subscriber children_sub_;
  ros::Subscriber peer_sub_;
  ros::Subscriber parent_sub_;

  // Node handler
  ros::NodeHandle pub_nh_;
  ros::NodeHandle sub_nh_;
  ros::NodeHandle local_;

  // Threads
  boost::thread *update_thread;
  boost::thread *work_thread;
  boost::thread *check_thread;

  // Mutex
  boost::mutex mut;
  boost::mutex work_mut;

  // Recording Mutex
  boost::thread *record_thread;

  // Conditional Variable
  boost::condition_variable cv;

  // Working state
  bool working;
};
}  // namespace task_net
#endif  // INCLUDE_NODE_H_