/*
 * Blocks.cpp
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/*
 * Example file showing a demo with 100 agents split in four groups initially
 * positioned in four corners of the environment. Each agent attempts to move to
 * other side of the environment through a narrow passage generated by four
 * obstacles. There is no roadmap to guide the agents around the obstacles.
 */

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif

#include <cmath>
#include <cstdlib>

#include <vector>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <fstream>

#include "orca/RVO.h"
#include <opencv2/opencv.hpp>

#define CV_FLAG true

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

// ###########################
/* Including the ros headers*/
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
// ###########################


/*File write data*/
//std::ofstream logg("src/ros_orca/orca/data/data_gazebo_pose.txt", std::ofstream::out | std::ofstream::trunc);
std::ofstream logg("src/ros_orca/orca/data/data_gazebo_pose.txt", std::ofstream::out | std::ofstream::trunc);

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

int r = 600;
int c = 600;

// ALL DIMENSIONS ARE IN CM

/* Circle at 300, 300   radius = 50 units*/
int center_x{0};
int center_y{0};
int divisionAngle{360};
double radiusOfObstacle{50};  // in pixels

std::string robot = "turtlebot3_waffle_pi";
std::string obs = "unit_cylinder";

int n_agents = 0;


cv::Mat outputIm1 = cv::Mat::zeros(cv::Size(r,c), CV_8UC3);	
std::string window_name = "Circle Simulation with Velocity 5 units, th(obs) = 5.0";


double robot_radius = 22;
double safety_buffer = 10;
double net_radius = robot_radius + safety_buffer;

cv::Mat outputIm;


RVO::Vector2 robot_current_position;

ros::Publisher pb;
void setupScenario(RVO::RVOSimulator *sim)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
	std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.12f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 5.0f, 50.0f, net_radius*1.0f, 22.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
	int i{0}, j{0};
	sim->addAgent(RVO::Vector2(-200.0f + i * 10.0f,  0.0f + j * 10.0f));
	goals.push_back(RVO::Vector2(200.0f, 00.0f));


	/*for (size_t i = 0; i < 5; ++i) {
		for (size_t j = 0; j < 5; ++j) {
			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, 75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, 75.0f));
		}
	}*/

	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

	/*      4   Square        */
	/*obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
	obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));*/

	
	
	for (int i=0; i<divisionAngle; i++){
		obstacle1.push_back(RVO::Vector2(-50 * sin(i*M_PI/180.0f), 50 * cos(i*M_PI/180.f) ));
		//std::cout << i <<" " << RVO::Vector2(-50 * sin(i*M_PI/180.0f), 50 * cos(i*M_PI/180.f) ) << "\n";
		//logg 	  << i <<" " << RVO::Vector2(-50 * sin(i*M_PI/180.0f), 50 * cos(i*M_PI/180.f) ) << "\n";
	}

	//std::cout << "Done printing obstacle \n";
	//logg	  << "Done printing obstacle \n";	
	
	sim->addObstacle(obstacle1);
	//sim->addObstacle(obstacle2);
	//sim->addObstacle(obstacle3);
	//sim->addObstacle(obstacle4);

	/* Process the obstacles so that they are accounted for in the simulation. */
	sim->processObstacles();
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim)
{
	/* Output the current global time. */
	std::cout << "GlobalTime: " << sim->getGlobalTime() << "\n";
	logg	  << "GlobalTime: " << sim->getGlobalTime() << "\n";

	cv::Vec3b red (0,0,255);
	cv::namedWindow(window_name);
	//std::cout << "Num of agents = " << sim->getNumAgents() << "\n";
	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector2 obj = sim->getAgentPosition(i);

		if(CV_FLAG){
			outputIm.at<cv::Vec3b>(300 + obj.y() ,300 + obj.x()) = red;
			cv::circle(outputIm, cv::Point(300 + obj.x(),300 + obj.y()),net_radius, cv::Scalar(255,0,0), -1, 8, 0);
			cv::circle(outputIm, cv::Point(300 + obj.x(),300 + obj.y()),robot_radius, cv::Scalar(0,0,255), -1, 8, 0);
		}

		std::cout << " " << obj << "\n";
	}
	
	if(CV_FLAG){
		cv::imshow(window_name, outputIm);
		cv::waitKey(1);
		outputIm = outputIm1.clone();
	}


	std::cout << std::endl;
}
#endif

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i); // get this from Gazebo
		//RVO::Vector2 goalVector = goals[i] - robot_current_position; // get this from Gazebo
		robot_current_position = RVO::Vector2((sim->getAgentPosition(i)).x()  , (sim->getAgentPosition(i)).y());  // other robotCurrPosition is inside callback function

		std::cout << "Robot Curr Position:  " << robot_current_position.x() << "   " << robot_current_position.y() << "\n";
		logg	  << "Robot Curr Position:  " << robot_current_position.x() << "   " << robot_current_position.y() << "\n";
		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector) * 5;
		}


		sim->setAgentPrefVelocity(i, goalVector);
		std::cout <<"Agent Velocity : " << sqrt(goalVector*goalVector) << "\n"; // Set this to CMD_VEL
		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;
	
		RVO::Vector2 agent_change_in_vel = dist * RVO::Vector2(std::cos(angle), std::sin(angle));
		
		RVO::Vector2 agent_pref_velocity = sim->getAgentPrefVelocity(i) + agent_change_in_vel;
		sim->setAgentPrefVelocity(i, agent_pref_velocity);

		std::cout << "Robot Curr Velocity:  " << agent_pref_velocity.x() << "   " << agent_pref_velocity.y() << "\n";
		logg	  << "Robot Curr Velocity:  " << agent_pref_velocity.x() << "   " << agent_pref_velocity.y() << "\n";


		geometry_msgs::Twist obg;
		obg.linear.x = agent_pref_velocity.y()*0.5;
		obg.linear.y = agent_pref_velocity.x()*0.5;
		//pb.publish(obg);
		
		ros::Duration(0.01).sleep();
	}
}

void model_callback_function(const gazebo_msgs::ModelStates::ConstPtr & ptr){
	int obs_index{0}, robot_index{0};
	for (int i=0; i<(ptr->name).size(); i++){
	//std::cout << (ptr->name)[i] << "\t";
	
	if((ptr->name)[i]==robot)
		robot_index = i;
	else if((ptr->name)[i]==obs)
		obs_index = i;
	}

	
	auto robot_data{(ptr->pose)[robot_index]};
	auto obs_data {(ptr->pose)[obs_index]};

	//std::cout << "Robot current location : ";
	//std::cout << robot_data.position.x << " " << robot_data.position.y << " " << robot_data.position.z << "\n";

	//robot_current_position = RVO::Vector2(robot_data.position.x*100, robot_data.position.y*100); 		// other robot_current_postion is inside callback function
	
	std::cout << "\n";
	logg << "\n";
	ros::Duration(0.0001).sleep();
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 20.0f * 20.0f) {
			return false;
		}
	}

	return true;
}

int main(int argc, char**argv)
{		
	//logg << "Jai Gurudev";
	//logg << "\nThank you\n";
	ros::init(argc, argv, "simple_obstacle_orca1");
	ros::NodeHandle nh;
	
	ros::Subscriber sh = nh.subscribe("/gazebo/model_states", 1, model_callback_function);
	/* Create a new simulator instance. */
	RVO::RVOSimulator *sim = new RVO::RVOSimulator();

	pb = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

	/* Set up the scenario. */
	setupScenario(sim);
	
	/*  4 Square Printing in Image*/
	/*cv::Point pt1r1(-40.0f + 300, 40.0f + 300);
	cv::Point pt2r1(-10.0f + 300, 10.0f + 300);
	cv::circle(outputIm1, pt1r1, 2, cv::Scalar(0,0,255),-1);
	cv::line(outputIm1, pt1r1 , pt2r1, cv::Scalar(255,0 ,0), 2 );

	cv::Point pt1r2(10.0f + 300, 40.0f + 300);
	cv::Point pt2r2(40.0f + 300, 10.0f + 300);

	cv::Point pt1r3(10.0f + 300, -10.0f + 300);
	cv::Point pt2r3(40.0f + 300, -40.0f + 300);

	cv::Point pt1r4(-40.0f + 300, -10.0f + 300);
	cv::Point pt2r4(-10.0f + 300, -40.0f + 300);

	cv::rectangle(outputIm1,pt1r1, pt2r1, cv::Scalar(30,255,255),  1);

	cv::rectangle(outputIm1, pt1r2, pt2r2, cv::Scalar(30,255,255), 1);

	cv::rectangle(outputIm1,pt1r3, pt2r3, cv::Scalar(30,255,255),  1);

	cv::rectangle(outputIm1, pt1r4, pt2r4, cv::Scalar(30,255,255), 1);*/

	/* Circular obstacle with dimensions "radiusOfObstacle" , "divisionAngle" */

	if(CV_FLAG){
		int num = sim->getNumObstacleVertices() ;
		for(int i=0; i< num ; i++){
			int curr_index = i%num;
			int next_index = (i+1)%num;
			
			RVO::Vector2 curr_vertex = sim->getObstacleVertex(curr_index);
			RVO::Vector2 next_vertex = sim->getObstacleVertex(next_index);

			cv::line(outputIm1, cv::Point(300+ curr_vertex.y(), 300 + curr_vertex.x()) , cv::Point(300+next_vertex.y(), 300+next_vertex.x()), cv::Scalar(30, 255, 255), 2 ); 
		}	
		
		outputIm = outputIm1.clone();

		cv::imshow(window_name, outputIm);
		cv::waitKey(1);
	}
	int count=0;

	/* Perform (and manipulate) the simulation. */
	do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
		updateVisualization(sim);
#endif
		setPreferredVelocities(sim);
		sim->doStep();
		ros::spinOnce();
		count++;
		std::cout << count <<"\n";
	}while (!reachedGoal(sim) && ros::ok()/*&& count<1000*/);
	
	
	//logg.close();	

	if(CV_FLAG){
		std::cout << outputIm.channels() << "\n";
		cv::waitKey(10);
		cv::destroyAllWindows();
	}

	/*Give stop command to the robot*/
	geometry_msgs::Twist obg;
	//pb.publish(obg);
	ros::Duration(0.01).sleep();

	
	delete sim;

	return 0;
}
