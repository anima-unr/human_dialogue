/*
remote_mutex
Copyright (C) 2015  Luke Fraser

remote_mutex is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

remote_mutex is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with remote_mutex.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "remote_mutex/remote_mutex.h"
#include "timeseries_recording_toolkit/record_timeseries_data_to_file.h"

class RemoteMutexService;

void Record(RemoteMutexService* mut);

class RemoteMutexService {
 public:
  bool locked;
  std::string owner;
  ros::ServiceServer service;
  ros::NodeHandle ns;
  float activation_potential;
  boost::mutex mut;
  boost::thread* record_thread;
  std::ofstream file;
  recording_toolkit::FilePrintRecorder record_object;

  explicit RemoteMutexService(const char* name)
      : record_object("/home/janelle/pr2_baxter_ws/src/Distributed_Collaborative_Task_Tree/Data/remote_mutex.csv",
        100) {
    locked = false;
    owner = "";
    activation_potential = 0.0f;
    service = ns.advertiseService(
      name,
      &RemoteMutexService::MutexRequest,
      this);
    record_thread = new boost::thread(&Record, this);
    record_object.StartRecord();;
  }

  ~RemoteMutexService() {
    delete record_thread;
    // record_object.WaitUntilFinishedWriting();
    record_object.StopRecord();
  }

  void RecordToFile() {
    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - time_t_epoch;
    double seconds = (double)diff.total_seconds() + (double)diff.fractional_seconds() / 1000000.0;
    record_object.RecordPrintf("%f, %s\n", seconds, owner.c_str());
}

  bool MutexRequest(remote_mutex::remote_mutex_msg::Request &req,
      remote_mutex::remote_mutex_msg::Response &res) {
    if (req.request) {
      ROS_INFO("asking for mutex lock [%f / %f] %s", activation_potential, req.activation_potential, req.node.c_str());

      if (locked) {
        res.success = false;
        ROS_DEBUG("Mutex Already Locked - Denied Access: %s", req.node.c_str());
      } else {
        // check if Nodes activation is the highest for a second
        if (activation_potential < req.activation_potential) {
          mut.lock();
          activation_potential = req.activation_potential;
          mut.unlock();
          boost::this_thread::sleep(boost::posix_time::millisec(100));
          if (activation_potential > req.activation_potential) {
            ROS_INFO("Not Highest Activation Potential - Denied Access: [%f / %f] %s", activation_potential, req.activation_potential, req.node.c_str());
            res.success = false;
          } else {
            mut.lock();
              locked = true;
              owner = req.node;
            mut.unlock();
              res.success = true;
              ROS_INFO("Mutex Locked - Granted Access: %s", req.node.c_str());
          }
        } else {
          ROS_INFO("Not Highest Activation Potential - Denied Access: [%f / %f]", activation_potential, req.activation_potential);
          res.success = false;
        }
      }
    } else {
      if (locked) {
        if (req.node == owner) {
          mut.lock();
          locked = false;
          owner = "";
          activation_potential = 0.0f;
          mut.unlock();
          res.success = true;
          ROS_INFO("Mutex Unlocked - Granted Access: %s", req.node.c_str());
        } else {
          res.success = false;
          ROS_INFO("Mutex Locked - Denied Access: %s", req.node.c_str());
        }
      } else {
        res.success = false;
        ROS_INFO("Mutex Already Unlocked: %s", req.node.c_str());
      }
    }
    return true;
  }
};

void Record(RemoteMutexService* mut) {
  while (true) {
    mut->RecordToFile();
    boost::this_thread::sleep(boost::posix_time::millisec(50));
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "remote_mutex_server");
  if (argc >= 2) {
    RemoteMutexService mutex(argv[1]);
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();
  } else {
    ROS_FATAL("A Mutex Name is a required Parameter - None Given");
    return -1;
  }
  return 0;
}
