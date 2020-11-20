
#ifndef TEST_SIMULATOR_1
#define TEST_SIMULATOR_1

#pragma once

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif


#include <cmath>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

/* Including the RVO headers*/
#include "orca/Definitions.h"
#include "orca/RVO.h"


/* Including the ros headers*/
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "orca_msgs/AgentState.h"

/*Set CV FLAG*/
#define CV_FLAG true




class Test_Sim{
	private:
	/* Saving into LOG Files*/
	std::ofstream logg;

	
	ros::NodeHandle nh_;	 	 /*Class Node handle*/
	
	ros::Publisher velocity_pb_;		 		/*Class Member publisher for Velocity*/
	
	ros::Subscriber model_state_sh_;		    /*Class Member subscriber for getting Gazebo Model States*/

    ros::Publisher my_robot_state_pb_;		 	/*Class Member publisher for publishing robot position*/

	ros::Subscriber static_obstacle_sh_;     	/* Class Member subscriber for obtaining static obstacle data*/

	void modelStatesCallbackFunction_(const gazebo_msgs::ModelStates::ConstPtr& );

	void staticObstaclesCallBackFunction_(const sensor_msgs::LaserScanConstPtr& );

    double T_robot_world[3][3];

    tf::TransformListener listener1_; 		// for base_footprint->map
    tf::StampedTransform transform1_; 		// for base_footprint->map

	tf::TransformListener listener2_; 		// for base_scan->map
    tf::StampedTransform transform2_; 		// for base_scan->map

    tf::Vector3 transformVelocity_(tf::Matrix3x3& mat, tf::Vector3& velInWorld);
	
	// ------------------------------------
	/* RVO Simulator Attributes & Methods*/

	//* Defining the variables for ORCA - START
	double robotRadius_{22.0f};
	double safetyBuffer_{5.0f};
	double netRobotRadius_{robotRadius_ + safetyBuffer_};
	float maxSpeed_{};
	RVO::Vector2 velocity;

	double timeHorizonAgent_{5.0f};
	double timeHorizonObstacle_{300.0f};

	float neighborDist_{};
	size_t maxNeighbors_{};
	//* Defining the variables for ORCA - END



	//* ORCA Env Setup - START

	// Creating a simulator instance 
	RVO::RVOSimulator *sim;

	// Setting up the scenario
	void setupScenario_();

	void setupAgent_();
	void setupObstacle_();

	// Method Update visualization (Increment Time Step dT)
	void updateVisualization_();

	// Setting up v_pref
	void setPreferredVelocities_();

	// Setup up robot
	std::string robotName_ {"turtlebot3_waffle"};
	RVO::Vector2 robotStart_{RVO::Vector2(-500.00f, 0.00f)};
	RVO::Vector2 robotGoal_{RVO::Vector2(600.00f, 300.00f)};

	// Vector to keep track of goals of each agent
	std::vector<RVO::Vector2> goals_;

	// Obstacle Data
	std::string obsName_ {"unit_cylinder"};
	double centerX_{0.00f}; 
	double centerY_{0.00f};
	int angularDivisions_{360};
	double radiusObstacle_{120.0f}; // in cm

	//* ORCA Env Setup - END


	//* Helper member functions and variables
	bool b_obstacleInitialized_{false};
	
	bool reachedGoal_();

	RVO::Vector2 robotCurrentPosition_{robotStart_};		
	std::vector<RVO::Vector2> obstData_;

    geometry_msgs::Twist toRobotFrame_(geometry_msgs::Twist&);  // Vel  [map -> base_footprint]
	
	void velocityPublisher_(const geometry_msgs::Twist& ); 		// alters velocity to ensure drift proof motion

	void dispTransformationMat_(tf::Matrix3x3& mat);			// print transfomation matrix

	public:
	Test_Sim();
	Test_Sim(ros::NodeHandle & nh);	
	~Test_Sim();

	RVO::Vector2 getRobotCurrentPosition();						// returns current robot position as RVO::Vector2
	RVO::Vector2 transformPointToMapFrame(tf::Matrix3x3& , const tf::Vector3& , const tf::Vector3& );
	
	//friend class Agent;
};



#endif
