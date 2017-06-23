#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <table_task_sim/Object.h>
#include <table_task_sim/Robot.h>
#include <table_task_sim/SimState.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <geometry_msgs/Pose.h>

#define OBJ_PFX 1000
#define ROB_PFX 2000
#define GOAL_PFX 3000

#define TOP_SPEED 0.1

table_task_sim::SimState simstate;

bool pick(table_task_sim::PickUpObject::Request  &req,
          table_task_sim::PickUpObject::Response &res)
{
  //res.result = table_task_sim::PickUpObject::SUCCESS;
	res.result = 0;

	// find object with the name to use
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		if ( req.object_name.compare( simstate.objects[i].name ) == 0 )
		{
			// object found 

			// set robot's goal to match object
			simstate.robots[req.robot_id].goal = simstate.objects[i].pose;
			float dist = 999;
			ros::Rate loop_rate( 10 );

			// wait for robot to reach goal
			do {
				float xdist = simstate.robots[req.robot_id].goal.position.x - simstate.robots[req.robot_id].pose.position.x;
				float ydist = simstate.robots[req.robot_id].goal.position.y - simstate.robots[req.robot_id].pose.position.y;
				dist = hypot(ydist,xdist);
				ROS_INFO ("robot [%d] moving to [%s] dist: %f", req.robot_id, req.object_name.c_str(), dist);
				loop_rate.sleep();

			} while( dist > 0.0001 );

			simstate.robots[req.robot_id].holding = req.object_name;

			return true;
		}
	}

	res.result = 1;
	return true;
}

bool place(table_task_sim::PlaceObject::Request		&req,
		   table_task_sim::PlaceObject::Response	&res)
{


	return true;
}

void publish_markers(ros::Publisher *mp)
{
	visualization_msgs::Marker marker;

	// these values are the same for all
	marker.header.frame_id = "/table";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;

	// publish table
	marker.id = 0001;
	marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.02;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

	mp->publish(marker);

	// publish object poses
	marker.type = visualization_msgs::Marker::CUBE;
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		marker.id = OBJ_PFX + i;
		marker.pose = simstate.objects[i].pose;
	    marker.scale = simstate.objects[i].scale;
	    marker.color = simstate.objects[i].color;

		mp->publish(marker);
	}

	// publish robot poses
	marker.type = visualization_msgs::Marker::CYLINDER;
	for( int i = 0; i < simstate.robots.size(); i++ )
	{
		marker.id = ROB_PFX + i;
		marker.pose = simstate.robots[i].pose;
	    marker.scale.x = 0.075;
	    marker.scale.y = 0.075;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;

		mp->publish(marker);		

		marker.id = GOAL_PFX + i;
		marker.pose = simstate.robots[i].goal;
	    marker.scale.x = 0.03;
	    marker.scale.y = 0.03;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;
	    marker.color.a = 0.5;

		mp->publish(marker);		

	}
}

void populate_state()
{
	table_task_sim::Object o;
	o.pose.position.x = 0.0;
	o.pose.position.y = 0.0;
	o.pose.position.z = 0.0;
	o.pose.orientation.x = 0.0;
	o.pose.orientation.y = 0.0;
	o.pose.orientation.z = 0.0;
	o.pose.orientation.w = 1.0;
	o.scale.x = 0.05;
	o.scale.y = 0.05;
	o.scale.z = 0.05;
	o.color.b = 1.0;
	o.color.a = 0.9;
	o.name = "meat";
	simstate.objects.push_back(o);

	o.name = "lettuce";
	o.pose.position.x = 0.25;
	o.color.r = 1.0;
	simstate.objects.push_back(o);

	table_task_sim::Robot r;
	r.pose.position.x = 0;
	r.pose.position.y = -0.45;
	r.pose.position.z = 0;
	r.color.r = 1.0;
	r.color.a = 0.9;
	r.goal = r.pose;
	simstate.robots.push_back(r);

	r.pose.position.y = 0.45;
	r.color.g = 0.5;
	r.goal = r.pose;
	r.goal.position.x = -0.5;
	simstate.robots.push_back(r);

}


int main(int argc, char* argv[] )
{
	ros::init(argc, argv, "table_sim");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);

	// populate data (TODO replace with file I/O)
	populate_state();

	// declare subscribers
	ros::ServiceServer pick_service = nh.advertiseService("pick_service", pick);

	// declare publishers
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("markers", 1000);
	ros::Publisher state_pub = nh.advertise<table_task_sim::SimState>("state", 1000);
	
	// async spinner thread
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	ros::Time last_iter = ros::Time::now();

	/* main control loop */
	while( ros::ok() )
	{
		ros::Time curr_time = ros::Time::now();

		// update end effector positions
		ros::Duration diff = curr_time - last_iter;
		for( int i = 0; i < simstate.robots.size(); i++ )
		{
			// get distance from current pos to goal
			float xdist = simstate.robots[i].goal.position.x - simstate.robots[i].pose.position.x;
			float ydist = simstate.robots[i].goal.position.y - simstate.robots[i].pose.position.y;
			float dist = hypot(ydist,xdist);
			float theta = atan2(ydist, xdist);

			float r = TOP_SPEED * diff.toSec();

			// if dist is less than top speed / dir
			if( dist < r )
			{
				// robot has reached goal
				simstate.robots[i].pose = simstate.robots[i].goal;
			}
			else
			{
				// move direction of travel top speed / dir
				simstate.robots[i].pose.position.x += r * cos(theta);
				simstate.robots[i].pose.position.y += r * sin(theta);
			}

			// if the robot is holding an object, move the object to where the robot is
			if( )

		}

		// publish markers
		publish_markers(&marker_pub);

		// publish current object and end effector positions 
		state_pub.publish(simstate);

		last_iter = curr_time;
		loop_rate.sleep();
	}


	return 0;
}