## About
This is the README file for Bug 1 and Bug 2 algorithms for fulfillment of Assignment 1 in CSCI 5980. The author of this code is Trevor Stephens (steph594@umn.edu)

## How to run
1.) Place the entire package (stephens_bug_algorithms) in a catkin workspace.  
2.) To run Bug 1 algorithm cd to the catkin workspace and use the command $ catkin_make && roslaunch stephens_bug_algorithms bug1.launch (To run Bug 2 algorithm cd to the catkin workspace and use the command $ catkin_make && roslaunch stephens_bug_algorithms bug2.launch)  
3.) Position the target (red block) and robot (blue block) in a desired starting location then type 'y' and press enter into the command line to start the algorithm.  
4.) At any time the target and/or robot can be repositioned to restart the algorithm  
5.) When the target is reached move the robot and target to restart the algorithm. It is advised to move the robot first as the algorithm will restart once the target has been moved. If the target is not moved then the algorithm will not restart.  
6.) Alternatively, to restart the algorithm the launch file can be relaunched as shown in step 2.  

## Comments
The algorithm has been tested on the maps provided in the bitmaps folder. To change to a new map navigate to the willow-erratic.world file and change the following line of code:  
bitmap "bitmaps/MAPNAMEHERE.png"  
by placing the desired map file name in the MAPNAMEHERE placeholder.  
  
There is currently a speedup of 4 in the willow-erratic.world file which helps simulations run faster. It has been tested at faster speedup values, but is suggested to keep it at 4 or lower. 


