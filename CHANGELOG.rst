^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Change logs for DynamicObstacleAvoidance Repository
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1 (2020-11-19) [Kavit]
-------------------
* Created DynamicObstacleAvoidance Repository
* Added env package - [world files and launch files]
* Added orca package - [contains ORCA library & additional files ]
* Added orca_msgs package - [contains the msg definitions used by orca package]


1.2 (2020-12-02) [Kavit]
-------------------
* Added CHANGELOG.rst file in outermost directory and every package in the directory
* In package orca : 
	* Updated test_sim.cpp file 
		- Made ORCA up and running with static world
		- Removed unnecessary print statements
		- Updated variable naming conventions
		- Updated coding style (camelCasing)
		- Created a new function runORCA_ which contains the loop for ORCA
	* Added test_sim_Functional_Information.txt which explains the API for test_sim.cpp
	* Added CHANGELOG.rst
* Added a new folder for animations


1.3(2020-12-06) [Kavit]
-------------------
* In package orca_msgs:
	* Converted "data" & "agent_ID" objects as arrays
* In package orca:
	* Updated CMakeLists.txt : 
		- Removed OPENCV_DIR tag
		- Only build executables map_to_odom_publisher & test_sim
		- Added add_dependency tag to test_sim executable
		- Raised the order of orca_msgs includes at the start of all header files using them
		- Made necessary changes to incorporate DetectedEntity arrays in Agent{.h & .cpp} , Test_Sim{.h & .cpp} files
* In package env:
	* Added new launch file for open_spaces with moving humans
* Solved Issue 1.1 which was pertaining to building the packages
* Added ReadMe.md


1.4(2020-12-21) [Kavit]
-------------------
* Updated orca (check changelogs ORCA)
* Updated env  (check changelogs env)


1.4(2020-12-21) [Kavit]
-------------------
* Updated orca 
	- Stopped using laser scans for static nav
	- Added Obstacle data manually for testing new method

1.5(2021-05-03) [Kavit]
-------------------
* Resolved build issue 
	- Created a new folder "additional_packages" 
	- Added person package to additional_packages/
