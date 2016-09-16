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

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <signal.h>
#include <vector>
#include <string>
#include <map>
#include "robotics_task_tree_eval/behavior.h"
#include "robotics_task_tree_eval/node_types.h"

void EndingFunc(int signal) {
  printf("Closing Program...\n");
  ros::shutdown();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "behavior_network", ros::init_options::NoSigintHandler);
  signal(SIGINT, EndingFunc);
  ros::NodeHandle nh("~");

  task_net::NodeId_t name_param;
  std::vector<std::string> peers_param_str;
  task_net::NodeList peers_param;
  std::vector<std::string> children_param_str;
  task_net::NodeList children_param;
  task_net::NodeId_t parent_param;

  if (nh.getParam("name", name_param.topic)) {
    printf("%s\n", name_param.topic.c_str());
  }
  if (nh.getParam("peers", peers_param_str)) {
    printf("%s\n", peers_param_str[0].c_str());
  }
  for (std::vector<std::string>::iterator it = peers_param_str.begin();
      it != peers_param_str.end(); ++it) {
    task_net::NodeId_t temp;
    temp.topic = *it;
    temp.pub = NULL;
    peers_param.push_back(temp);
  }
  if (nh.getParam("children", children_param_str)) {
    printf("%s\n", children_param_str[0].c_str());
  }
  for (std::vector<std::string>::iterator it = children_param_str.begin();
      it != children_param_str.end(); ++it) {
    task_net::NodeId_t temp;
    temp.topic = *it;
    temp.pub = NULL;
    children_param.push_back(temp);
  }
  if (nh.getParam("parent", parent_param.topic)) {
    printf("%s\n", parent_param.topic.c_str());
  }

  task_net::State_t state;
  task_net::Node * test;
  test = new task_net::ThenBehavior(name_param,
                            peers_param,
                            children_param,
                            parent_param,
                            state,
                            false);
  ros::spin();
  delete test;
  return 0;
}
