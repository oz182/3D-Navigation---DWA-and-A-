
## This project is a recreation of the method suggested in the paper: "Global Dynamic Path Planning Fusion Algorithm Combining Jump A Algorithm"



https://user-images.githubusercontent.com/91877982/217909303-61b3c168-a74f-4850-8c20-bb44fe6a11c4.mp4





#The fusion algorithm concept

As described above, the purpose of the algorithm is to combine between global and local path planning algorithms. The general idea of this concept can be seen in the following chart:


<img width="217" alt="image" src="https://user-images.githubusercontent.com/91877982/217907672-f9ff3986-4ca4-4868-ad93-0716b8b72c54.png">










In the above chart, one can understand that the A* algorithm serves as the base for the “dynamic window approach” algorithm, as it need to follow nodes along the A* path. It is important to note that the A* path is calculated single time in the beginning of the algorithm, while the Dynamic window algorithm constantly recalculates the best trajectory to move along.

In order to maintain the new path instead of moving towards the target, the researchers has modified the evaluation function:

G(v,w)=α∙AstarHead(v,w)+β∙dist(v,w)+γ∙vel(v,w)

As one can see, the new heading part of the equation has replaced with a part that programmed to follow the A* path’s nodes. The results, gives a smoother path, as well as shortest path.


##Simulation of 3D DWA and A* using MATLAB

In this part of the paper, I will describe in detail about my code and the functions I wrote to execute the simulation of the algorithm from the research paper. In addition to simulate the algorithm from the paper, I’ve enlarged it to operate in a 3-dimentaional space, Which I will cover in the following chapter.

3D DWA environment - For making the simulation to work in a 3D configuration space, there were two main modifications I needed to add to the regular DWA algorithm. I chose to treat this robot as a missile, so it is now has the ability to move only forward in the space according to the nose direction. The missile’s nose can move in yaw and pitch to make its way in the space. Therefore, the first modification, is adding another axis to the dynamic window. So instead of containing only the forward velocity and yaw velocity, the dynamic window now also contains the pitch velocity. So, it (the dynamic window) can be imaged as a 3-axis space.

The next modification in the algorithm, is to add to the evaluation function the heading towards the target in the YZ plane. So, the new evaluation function is now look like this:

G(v,w,w_z )=α∙heading(v,w,w_z )+β∙dist(v,w,w_z )+γ∙vel(v,w,w_z )+δ∙headingZ(v,w,w_z)

Configurations section - The first section of the code (Configuration), contains all the data and definitions that are necessary for the algorithm to properly working. Along the things that located in this section are: Obstacle matrix (x,y,z of each obstacle), the robot’s dynamic constrains, the current state vector definition, time interval value, evaluation factors, etc.

Main loop – The main loop is the running center of the simulation. It has three main roles: 

•	AstarPlan (Line 50) - It calls the A* function and outputs a to column vector of the path coordinates x and y (Not in a loop, calls only once). 
  At first, we create an “occupancy map” – a map with obstacles, using the function “setOccupancy” (built-in). Then I’ve built a “planner” which is an     object contains the logic of the path planning (Using a built-in function “plannerAStarGrid”). Then we can call the function “plan” for planning the     path from start point to an end point.
  
•	Function “f” (Actual movement) (Line 67) – calculates the new state vector according to the speed for the next time interval.
  With the calculation above, the current state vector (red), gets added with the selected velocities vector (blue) that multiply by the time interval.     This gives out a new vector which is the new state vector.
    
•	DynamicWindowAprroch (Line 65) - it calls the DWA function and outputs the selected velocities to the next movement

•	“CalcDynamicWindow” – As it names suggests, this function calculates the dynamic window of the robot. It does that using the method described above (page 4 and 5). The function outputs a vector that contains the allowed speeds (min and max values of forward, horizontal, and vertical speeds).

•	“Evaluation” – this function, goes over every allowed speed in the dynamic window (with a preset resolution), and for every speed, it calls to the evaluation functions (dist, heading and heading in Z). the output is a vector which contains the values of each part of the evaluation function.

•	“NormalizeEval” – This is a simple function that normalize the values of each part of the evaluation function.

•	“GenerateTrajectory” – This function crates a trajectory towards the movement direction of the robot, that is later being plotted. This function is not working properly so I turned off the display of it.

•	“CalcHeadingEval” & “CalcDistFromZ” – are both functions that works in the same way for the same purpose but on different planes (one on XY and the other on YZ). Those functions calculate the degrees towards the target. If the robot moves directly towards the target (means the angle from the robot trajectory to the goal is 0) then it gets 180 which is the maximum score it can gets. The chosen direction is the one with the highest score.

•	“CalcDistEval” – Checks for the distance between the robot from the near obstacles. As far the robot from an obstacle, the higher the score (used for the evaluation function). The distance is simply calculated by a 3D “Normal” vector between the robot and position and the obstacle position with addition of the obstacle’s radius (can be preset in the configuration section).

•	“ClacBreakingDist” – A function that constantly calculates the break distance of the robot (the distance that would take the robot to get to 0 velocity). This distance than used to check whether the robot can stop before colliding with the next chosen speed by the evaluation function.


#Conclusions

It can be seen in the result below, that the DWA algorithm serve as a good option in an unknown environment, and especially when it is needed to take in account the dynamical constrains of the robot (such as autonomous cars, Cruz missiles, etc.). When it gets combined with a global planning algorithm, as was suggested in the article and done in this simulation, it is easy to see that the robot does follow the shortest path of the A* algorithm and gives the DWA another plane of autonomy.

<img width="319" alt="image" src="https://user-images.githubusercontent.com/91877982/217908839-cbbc4851-6107-4d57-9dda-7ecd44949c5e.png">


In top view:
<img width="298" alt="image" src="https://user-images.githubusercontent.com/91877982/217908956-6f8e60e6-7f4e-4cb5-95f3-c1972d32186a.png">
