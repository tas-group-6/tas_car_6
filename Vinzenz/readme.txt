Documentation 
TAS Project
Vinzenz Dallabetta 


Velocity Control: 

tas_autonomous_control_node.cpp
-	This node is the expanded version of the forma tas_autonomous_control_node.cpp
-	Added Laserscan subscriber
-	Added AMCL position subscriber  
-	Added two different stat machines which are controlling the velocity of the car 

Laserscan subscriber
-	Calculate the range to the next objects in front of the car and sets according to that a global 
	flag for the velocity control [NO_FREE_SPACE, CORRIDOR, ALL_FREE]
-	Furthermore a velocity gain factor is calculated based on the range and the global flag 

AMCL position subscriber
-	Detection of corners based on the car's position
-	Based on the position of the car in the corner several velocity modifications are done while the
	car is driving around a corner
-	Furthermore, a global Flag is set according to the car's position in the corner [NO_CORNER, 	
	APP_CORNER, LEAVE_CORNER, EXIT_CORNER] for the velocity control

Main function
-	Read in the recorded corner points 
-	Implement the two subscriber
-	Translate the trajectory's, given by the local planner and the velocity, given by the two 
	subscriber to a PWM signals 

Cornerpionts.txt 
-	Recorded corner points  
-	Position of the corner point [X,Y,Z] in [meter]
-	Orientation of the corner piont [as quaternion]
-	Alpha correction rotation of the map [an degree] 
-	Leave position [meter]
-	Exit position [meter]

