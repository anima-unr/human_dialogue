# Distributed_Collaborative_Task_Tree
Clone of Luke's code as of Sep 15, 2016

## Installation
1. Clone this repository and put it in the /src folder of your catkin workspace.
2. Install the following dependencies:
- [GPD](https://github.com/atenpas/gpd): place this in catkin/src/
- [YOLO](https://github.com/leggedrobotics/darknet_ros): You'll need to setup an SSH key to install this. **Don't** just clone the repository. Then place this in catkin/src/
- Graphics Card Driver: You need to install your graphics card driver which can be found [here](http://www.nvidia.com/Download/index.aspx) if it is not already installed. Otherwise when installing Cuda(below) you may experience an infinite logout loop in Ubuntu 14.04LTS due to a missing graphics card driver. 
- Cuda:install the right version of cuda with respect to your graphics card which can be found [here](https://developer.nvidia.com/cuda-80-download-archive).
3. In your catkin_ws, `source devel/setup.bash`, then `catkin_make`. If catkin fails to make then install the missing depenedencies.
## Potential Problems
- You may need to install python bindings for ROS.
- If you do install your graphics card driver but are still experiencing an infinite logout loop then you may need to update your kernal since the Spectre and Meltdown patches were added since the graphics card drivers and Cuda are written for the updated kernal. You can do this by doing a full update.
- If Darknet/Yolo fails to compile then you may need to modify the CMakeLists.txt file directly under `${CUDA_NVCC_FLAGS};` add `--std=c++11`.
- If GPD fails to compile then you may need to modify the CMakeLists.txt file by adding `add_dependencies(${PROJECT_NAME}_detect_grasps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})` after the `## Declare C++ executables` or the line `add_executable(${PROJECT_NAME}_test_occlusion src/tests/test_occlusion.cpp)`
## Running the Multi-Robot Simulator
Inside the Distributed_Collaborative_Task_Tree there is a `ros_sim.sh` file that runs the whole simulator. Feel free to modify it as need be. I'd reccomend moving it to your home directory as this will make it easier to run.
### Dropping an Object in Simulation
When the simulator is running:
1. Open a new terminal
2. `source devel/setup.bash`
3. When the robot is holding an object (the blue circle and square are moving together) run `rosservice call /drop_service "robot_id: 0"` and the object should be dropped (the bluce circle continues moving without the square).

