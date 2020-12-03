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
