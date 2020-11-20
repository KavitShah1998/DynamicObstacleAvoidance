/*
 * This file provides an API to interact with the RVO2 (ORCA) Library
 * which has been modified to work in ROS
 * 
 */

#include "orca/test_sim.h"



/*
 * Constructor 
 * Input : NodeHandle
 * 
 * 
 * Process Flow :  
 * 1.  Wait till 1st instance of laser scan is received
 * 2.  Once step1 is done, initialize the publishers and subscribers, setup RVO::Simulator & setup the agent(s)
 * 3.  Loop to execute ORCA
 */
Test_Sim::Test_Sim(ros::NodeHandle& nh) : nh_(nh){
	logg = std::ofstream("src/ros_orca/orca/data/data_gazebo_pose.txt", std::ofstream::out | std::ofstream::trunc);

	std::cout << "Test_Sim Constructor \n";
	logg << "Test_Sim Constructor \n";

	static_obstacle_sh_ = nh_.subscribe("/scan" , 1, &Test_Sim::staticObstaclesCallBackFunction_,this);
	
	while(!b_obstacleInitialized_){
		ROS_INFO("Waiting to Receive Laser Scan CallBacks");
		ros::Duration(1.0).sleep();
		ros::spinOnce();
	}

	velocity_pb_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

	model_state_sh_ = nh_.subscribe("/gazebo/model_states", 1, &Test_Sim::modelStatesCallbackFunction_, this);

	my_robot_state_pb_ = nh_.advertise<orca_msgs::AgentState>("/my_robot/modelStates",10);

	sim = new RVO::RVOSimulator();

	setupScenario_();
	

	int count{0};

	// ORCA main loop
	do {
		
		// setup obstacles in every loop if working in dynamic environments
		setupObstacle_();



#if RVO_OUTPUT_TIME_AND_POSITIONS
		// just to print data
		updateVisualization_();
#endif


		setPreferredVelocities_();

		// obstain the necessary transform between frames

		try{  // listener for transforming vel from map frame ro robot frame
			listener1_.waitForTransform("/base_footprint", "/map", 
			ros::Time(0), ros::Duration(10.0));
			
			listener1_.lookupTransform("/base_footprint", "/map",
			ros::Time(0), transform1_);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// computes new agent velocities
		sim->doStep();


		RVO::Vector2 new_agent_vel_after_doStep = sim->getAgentVelocity(0);
		// std::cout << " New agent velocity after doStep = " << new_agent_vel_after_doStep <<"\n";

		/*Important*/
		//agent_pref_velocities are in the global frame and need to converted into the frame of the robot

		geometry_msgs::Twist agent_vel_in_world;
		agent_vel_in_world.linear.x = new_agent_vel_after_doStep.x();
		agent_vel_in_world.linear.y = new_agent_vel_after_doStep.y();

		auto agent_vel_in_self_frame = toRobotFrame_(agent_vel_in_world);
		// std::cout << "Robot Altered Velocity:  " << agent_vel_in_self_frame.linear.x << "   " << agent_vel_in_self_frame.linear.y << " " << agent_vel_in_self_frame.angular.z << "\n";


		velocityPublisher_(agent_vel_in_self_frame);

		ros::spinOnce();
		// count++;
		// std::cout << "count =" << count <<"\n";
	}while (!reachedGoal_() && ros::ok());


}

/*
 * Destructor  
 * 
 * Process Flow :  
 * 1.  Publish 0 velocity fir 0.1 secs to ensure that robot stops after cycle completion
 */
Test_Sim::~Test_Sim(){
	std::cout << "Test_Sim Destructor \n";
	// logg << "Test_Sim Destructor \n";
	int count{10};
	while(--count>0 && ros::ok()){
		geometry_msgs::Twist zeroVelocity;
		velocity_pb_.publish(zeroVelocity);
		ros::Duration(0.01).sleep();
	}

	delete sim;

}


/*
 * setupAgent_()
 * Input : -
 * Output : void
 * 
 * Process Flow :  
 * 1.  Initialise the default agent attributes. Add the start adn goal for agent
 */
void Test_Sim::setupAgent_(){

	std::cout << "Setting up agent\n";
	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, timeHorizonAgent_, timeHorizonObstacle_, netRobotRadius_, 22.0f);

	// Adding Robot as an agent
	sim->addAgent(robotStart_);
	goals_.push_back(robotGoal_);

}

/*
 * setupObstacle_() 
 * Input : -
 * Output : void
 * 
 * Process Flow :  
 * 1.  Wait till 1st instance of laser scan is received
 * 2.  Once step1 is done, initialize the publishers and subscribers, setup RVO::Simulator & setup the agent(s)
 */
void Test_Sim::setupObstacle_(){

	std::cout << "Setting up obstacle\n";

	sim->clearObstacleVector();

	auto t1 = ros::Time::now();
	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */

	/* std::vector<RVO::Vector2> obstacle1, obstacle2; // a circular obstacle at the origin or radius 50 cm

	for (int i{0}; i < angularDivisions_; i++){

		obstacle1.push_back(RVO::Vector2(radiusObstacle_ * cos(i*M_PI/180.0f), radiusObstacle_ * sin(i*M_PI/180.0f) ));
		obstacle2.push_back(RVO::Vector2(300 + 100 * cos(i*M_PI/180.0f), 300 + 100 * sin(i*M_PI/180.0f)));

	}
	// std::cout << "Size of Obstacle 1 : " << obstacle1.size() << "\n";
	// std::cout << "Size of Obstacle 2 : " << obstacle2.size() << "\n";

	for (int i{0}; i <= angularDivisions_; i++){
		obstacle1.push_back(RVO::Vector2(-50.0 * sin(i*M_PI/180.0f), 50.0 * cos(i*M_PI/180.f) ));
		obstacle2.push_back(RVO::Vector2(radiusObstacle_ * cos(i*M_PI/180.0f), radiusObstacle_ * sin(i*M_PI/180.0f) ));


	}

	// Add the obstacle data into the RVOSimulator object
	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	// .... add more obstacles if any _ here _*/


	// obstData_ is a member function of the class which is std::vector<Vector2>
	sim->addObstacle(obstData_);

	/* Process the obstacles so that they are accounted for in the simulation. */
	sim->processObstacles();

	std::cout << "Time to execute setupObstacle : " << (ros::Time::now()-t1).toSec() <<"\n";
}


/*
 * setupScenario_()
 * Input : -
 * Output : void
 * 
 * Process Flow :  
 * 1.  Setup time_step, agent and obstacle data for beginning simulation
 */
void Test_Sim::setupScenario_()
{
	std::cout << "setupScenario \n";
	// logg << "setupScenario \n";

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
	std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.2f);

	/* Setup agent attributes*/
	setupAgent_();

	/* Setup obstacle points from laser scan*/
	setupObstacle_();
	
}


#if RVO_OUTPUT_TIME_AND_POSITIONS
void Test_Sim::updateVisualization_()
{
	std::cout << "updateVisualization \n";
	logg << "updateVisualization \n";

	/* Output the current global time. */
	std::cout << "GlobalTime: " << sim->getGlobalTime() << "\n";
	//logg	  << "GlobalTime: " << sim->getGlobalTime() << "\n";

	
	//std::cout << "Num of agents = " << sim->getNumAgents() << "\n";
	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector2 agent_position = sim->getAgentPosition(i);
		std::cout << " " << agent_position << "\n";
	}

	std::cout << std::endl;
}
#endif


/*
 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
 * direction of the goal.
 */

void Test_Sim::setPreferredVelocities_()
{
	std::cout << " setPreferredVelocitties \n";
	logg << " setPreferredVelocitties \n";


#ifdef _OPENMP
#pragma omp parallel for
#endif

	for (int i = 0; i <static_cast<int>(sim->getNumAgents()); ++i) {
		
		//RVO::Vector2 goalVector = goals_[i] - robotCurrentPosition_; // get this from Gazebo
		RVO::Vector2 goalVector = goals_[i] - sim->getAgentPosition(i);
		std::cout << "Robot Goal Vector: " << goalVector << "\n";
		// std::cout << "Gazebo Position:  " << robotCurrentPosition_.x() << "   " << robotCurrentPosition_.y() << "\n";
		// std::cout << "RVOSim Position:  " << sim->getAgentPosition(i).x() << "   " << sim->getAgentPosition(i).y() << "\n";
		// logg << "Gazebo Position:  " << robotCurrentPosition_.x() << "   " << robotCurrentPosition_.y() << "\n";
		// logg << "RVOSim Position:  " << sim->getAgentPosition(i).x() << "   " << sim->getAgentPosition(i).y() << "\n";
		//logg	  << "Robot Curr Position:  " << robot_current_position.x() << "   " << robot_current_position.y() << "\n";
		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		} // converted it into a unit vector


		sim->setAgentPrefVelocity(i, goalVector);
		//std::cout <<"Agent Velocity : " << sqrt(goalVector*goalVector) << "\n"; // Set this to CMD_VEL
		
		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;
	
		RVO::Vector2 agent_change_in_vel = dist * RVO::Vector2(std::cos(angle), std::sin(angle));
		
		RVO::Vector2 agent_pref_velocity = sim->getAgentPrefVelocity(i) + agent_change_in_vel;
		sim->setAgentPrefVelocity(i, agent_pref_velocity);

		std::cout << "Velocity Before Transformation:  " << agent_pref_velocity.x() << "   " << agent_pref_velocity.y() << "\n";
		// logg	  << "Velocity Before Transformation:  " << agent_pref_velocity.x() << "   " << agent_pref_velocity.y() << "\n";


	}
}


/*
 * getRobotCurrentPosition()
 * Input : -
 * Output : void
 * 
 * Process Flow :  
 * 1.  returns current robot position
 * 
 *     this function is used in agent.cpp
 * 	   currently unused
 */
RVO::Vector2 Test_Sim::getRobotCurrentPosition(){
	return robotCurrentPosition_;
} 



/*
 * modelStatesCallbackFunction_()
 * Input : const ptr to gazebo_msgs::ModelStates
 * Output : void
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Get index for robot and obstacle from model_states_publisher
 * 2.  Publish robot position and velocity on /my_robot/modelStates
 */
void Test_Sim::modelStatesCallbackFunction_(const gazebo_msgs::ModelStates::ConstPtr& modelStatePtr){
	std::cout << "Model _States _Callback _Function \n";
	logg << "Model _States _Callback _Function \n";

	int obs_index{0}, robot_index{0};
	for (int i=0; i<(modelStatePtr->name).size(); i++){	
		if( (modelStatePtr->name)[i]== robotName_ )
			robot_index = i;
		else if( (modelStatePtr->name)[i]== obsName_ )
			obs_index = i;
	}

	
	auto robot_data{(modelStatePtr->pose)[robot_index]};
	auto obs_data {(modelStatePtr->pose)[obs_index]};

	//std::cout << "Robot current location : ";
	//std::cout << robot_data.position.x << " " << robot_data.position.y << " " << robot_data.position.z << "\n";

	robotCurrentPosition_ = RVO::Vector2(robot_data.position.x*100.0f, robot_data.position.y*100.0f); 		// The data obtained is in meters, multiply by 100 to convert it into centimeters
	RVO::Vector2 robotVelocity = sim->getAgentVelocity(0);
	orca_msgs::AgentState robotState;

	robotState.header.stamp = ros::Time::now();
	robotState.agent_ID = 0;
	robotState.data.pos.x = robotCurrentPosition_.x();
	robotState.data.pos.y = robotCurrentPosition_.y();
	robotState.data.vel.x = robotVelocity.x();
	robotState.data.vel.y = robotVelocity.y();
	robotState.data.radius = netRobotRadius_;
	my_robot_state_pb_.publish(robotState);


	ros::Duration(0.0001).sleep();
}



/*
 * staticObstaclesCallBackFunction_()
 * Input : const ptr to sensor_msgs::LaserScan
 * Output : void
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Setup tf transform & tf listener to get transformation from /base_scan to /map frames
 * 2.  For every scan line in (r, theta) -> Convert it into (x,y) which is in robot's /base_scan
 * 	   -> convert the (x,y) from /base_scan to /map frame as a Vector2 object
 * 3.  Add the Vector2 object to std::vector<Vector> obstData_ which will be passed into simulation
 * 	   from addObstacle()
 */

void Test_Sim::staticObstaclesCallBackFunction_(const sensor_msgs::LaserScanConstPtr& scans){

	size_t size = scans->ranges.size();
	ROS_INFO("Laser SCAN CALL BACK RECVD");
	obstData_.clear();

    try{
        listener2_.waitForTransform("/map", "/base_scan", 
        ros::Time(0), ros::Duration(10.0));
        
        listener2_.lookupTransform("/map", "/base_scan",
        ros::Time(0), transform2_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    tf::Matrix3x3 mat = transform2_.getBasis();
    tf::Vector3 origin{transform2_.getOrigin()};
    
    for(int i=0;i<size; i++){

		if(!isinf(scans->ranges[i])){
			double x = (scans->ranges[i]) * cos(i*M_PI/180.0f);
			double y = (scans->ranges[i]) * sin(i*M_PI/180.0f);
			RVO::Vector2 point = transformPointToMapFrame(mat, origin, tf::Vector3(x,y,0));

			obstData_.emplace_back(point);
		}

    } 

	b_obstacleInitialized_ = true;

}

/*
 * transformPointToMapFrame()
 * Input : tf::Matrix3x3 [trasformation matrix from /base_scan to /map ], tf::Vector3 [origin of /map in /base_scan] , tf::Vector3 [point to be transformed]
 * Output : RVO::Vector2 [point in /map frame]
 * PUBLIC MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Convert the tf::Vector3 point from robot frame to /map frame and return as RVO::Vector2 object
 */
RVO::Vector2 Test_Sim::transformPointToMapFrame(tf::Matrix3x3& mat, const tf::Vector3& origin  , const tf::Vector3& point){
	return RVO::Vector2
                	(mat.getColumn(0).getX()*point.getX() + mat.getColumn(1).getX()*point.getY() + mat.getColumn(2).getX()*point.getZ() + origin.getX()*1,
					mat.getColumn(0).getY()*point.getX() + mat.getColumn(1).getY()*point.getY() + mat.getColumn(2).getY()*point.getZ() + origin.getY()*1);


}

/*
 * reachedGoal_()
 * Input : -
 * Output : bool
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  If all agents have reached their goals return true, else return false
 * 
 * THIS FUNCTION MIGHT NEED SOME TWEAKING AS WE MIGHT HAVE TO GIVE ROBOT MULTIPLE GOALS
 */
bool Test_Sim::reachedGoal_()
{
	std::cout << "reachedGoal_ \n";
	logg << "reachedGoal_ \n";

	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals_[i]) > 20.0f * 20.0f) {
			return false;
		}
	}

	return true;
}


/*
 * toRobotFrame_()
 * Input : geometry_msgs::Twist&
 * Output : geometry_msgs::Twist
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Using transformation between /map and /base_footprint, transfrom velocity to base_footprint
 * 2.  Convert the tf::vector3 robot velocity into geometry_msgs::Twist
 * 
 */
geometry_msgs::Twist Test_Sim::toRobotFrame_(geometry_msgs::Twist& msg){
	
	tf::Vector3 velWorldFrame = tf::Vector3(msg.linear.x, msg.linear.y, 0);
	tf::Matrix3x3 mat = transform1_.getBasis();
	tf::Vector3 velRobotFrame = transformVelocity_(mat, velWorldFrame);

	/* // Printing the Transformation Matrix from /map to /base_footprint*/
	// dispTransformationMat(mat);
	
	// finally we need to convert the vx, vy into omega and vx because that is how TUrtlebots work -\/-
	geometry_msgs::Twist velRobot;

	// RVO::Vector2 printObg = RVO::Vector2(velRobotFrame.getX(), velRobotFrame.getY()) ;
	// std::cout << "Velocity After Transformation:  " << printObg << " " << velRobotFrame.getZ() << "\n"; 

	velRobot.angular.z = 0.8 * atan2(velRobotFrame.getY(),
                                    velRobotFrame.getX());
	
	double lin_vel_scalar_multiplier {0.8};
	if(velRobot.angular.z > 0.5)
		lin_vel_scalar_multiplier = 0.3;
    
	velRobot.linear.x = lin_vel_scalar_multiplier * sqrt(pow(velRobotFrame.getX(), 2) +
                                  pow(velRobotFrame.getY(), 2));

	return velRobot;
}


/*
 * dispTransformationMat_()
 * Input : tf::Matrix3x3
 * Output : void
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Display the tf::Matrix3x3 type of transformation matrix
 * 
 */
void Test_Sim::dispTransformationMat_(tf::Matrix3x3& mat){
	
	std::cout << "Matrix = " << "\n \t" ;
	std::cout << (mat.getColumn(0)).getX() << " \t" <<(mat.getColumn(1)).getX() << " \t" << (mat.getColumn(2)).getX() << "\t"<< transform1_.getOrigin().getX() << "\n\t";
	std::cout << (mat.getColumn(0)).getY() << " \t" <<(mat.getColumn(1)).getY() << " \t" << (mat.getColumn(2)).getY() << "\t"<< transform1_.getOrigin().getY() << "\n\t";
	std::cout << (mat.getColumn(0)).getZ() << " \t" <<(mat.getColumn(1)).getZ() << " \t" << (mat.getColumn(2)).getZ() << "\t"<< transform1_.getOrigin().getZ() << "\n";
}


/*
 * velocityPublisher_()
 * Input : geometry_msgs::Twist&
 * Output : void
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Modify the control velocities (v_x & w_z) to ensure no drift
 * 2.  Publish the robot velocities with 0.1s sleep
 */
void Test_Sim::velocityPublisher_(const geometry_msgs::Twist& agentVel){
		

	geometry_msgs::Twist velocity;

	if(fabs(agentVel.angular.z) > 1){
		velocity.linear.x = 0.01;
		velocity.angular.z = 0.4 * (agentVel.angular.z) / fabs(agentVel.angular.z); // to capture the sense of rotation
	}
	else{
		velocity.angular.z = (agentVel.angular.z > 0.4) ? 0.4 : agentVel.angular.z;
		//velocity.linear.x = (agentVel.linear.x > 0.3) ? 0.3 : agentVel.linear.x;
		velocity.linear.x = (agentVel.linear.x > 0.4) ? 0.4 : agentVel.linear.x;

	}

	velocity_pb_.publish(velocity);
	
	
	ros::Duration(0.10).sleep();
	
}



/*
 * transformVelocity()
 * Input : tf::Matrix3x3 [trasformation matrix from /map to /base_footprint ], tf::Vector3 [velocity to be transformed]
 * Output : tf::Vector3
 * PRIVATE MEMBER FUNCTION
 * 
 * 
 * Process Flow :  
 * 1.  Using transformation between /map and /base_footprint, transfrom velocity to base_footprint
 * 2.  return tf::Vector3 velocity object
 * 
 */
tf::Vector3 Test_Sim::transformVelocity_(tf::Matrix3x3& mat, tf::Vector3& velInWorld){
	return tf::Vector3(mat.getColumn(0).getX()*velInWorld.getX() + mat.getColumn(1).getX()*velInWorld.getY() + mat.getColumn(2).getX()*velInWorld.getZ(),
				   mat.getColumn(0).getY()*velInWorld.getX() + mat.getColumn(1).getY()*velInWorld.getY() + mat.getColumn(2).getY()*velInWorld.getZ(),
				   mat.getColumn(0).getZ()*velInWorld.getX() + mat.getColumn(1).getZ()*velInWorld.getY() + mat.getColumn(2).getZ()*velInWorld.getZ());
}



int main(int argc, char**argv)
{		
	std::cout << "Int Main \n";
	ros::init(argc, argv, "simple_obstacle_orca1");
	ros::NodeHandle nh;

	Test_Sim tsim1(nh);

	return 0;
}
