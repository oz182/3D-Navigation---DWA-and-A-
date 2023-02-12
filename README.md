**Notice** – All the MATLAB and Simulink code was made with version 2020b. there might be problems running the files on an older/newer version of MATLAB.

**Part 1 – Path planning algorithm based on a research paper**

**The research paper -** “Global Dynamic Path Planning Fusion Algorithm Combining Jump-A Algorithm and Dynamic Window Approach”

**Description and contributions**

In this paper, the researchers introduce a new method for combining two path planning algorithms to get a fusion version with better performance (This improvement can be mainly seen as a global optimization and path smoothness).

The core of the algorithm is based on the “dynamic window approach” algorithm, which then being integrated with the A\* global path planning. This fusion, outputs a safe, global optimal path with high smoothness.

In more detailed manner, the new fusion algorithm creates not only the shortest path to the target (which is also considered as safe in terms of the robot’s dynamics capabilities) but also gives a smooth path which fits to the more natural and fluids movements.

**Separate description of each algorithm (A\* & DWA)**

For better understanding the entire scope of work that made in this paper, it is important to understand the principles of each path planning algorithm that used in this paper:

1.  A\* algorithm – A heuristic search algorithm used to calculate the optimal path in a static two-dimensional configuration space. It searches the optimal path node through the cost evaluation function as the next node to be traversed and repeats this process until the target point is found to form the optimal path.

    The cost evaluation function is as follows: . where is the cost evaluation function of the current position , represents the actual cost of the robot from the initial position to the current position , and represents the estimated cost of the robot from the current position to the target position. The cost can be thought as a distance. The algorithm aims to choose the next node with the lowest .

    ![](media/515d6266f05094d0b7253a23d923c07b.png)

    As can be seen in the example above, the green nodes are nodes that haven’t been explored, the red nodes are explored nodes, and the blue nodes are the selected nodes for the path. In each node, the centered large number is the , the left corner number is the , and the right corner number is the .

    Straight distances are taken to be 10 and thus the diagonal distances can be taken to be 14 (as in 90 degrees tringle).

It is important to mention that although the algorithm most likely to find the optimal path, it sometimes can be power consuming due to search in non-relevant directions (calculates the evaluation function of all the nearby nodes).

1.  DWA algorithm – A local path planning algorithm (unknown environments, sensor based), which not only navigate to the target without crash into obstacles, but also does that while taking in account the dynamic constrains of the robot (Rotational and straight velocities and accelerations). The algorithm is composed for the following main parts:
-   **Creating the dynamic window** – The dynamic window consist of all the velocities that the robot can do in the next time interval, with taking in account the distance to obstacles and the dynamic constrains of the robot.

    ![](media/2a4d6bbc10681f290a5cf7acafd7233e.png)

    In the figure above, you can find an illustration of the construction of the search space. When and axis are and (rotational and straight velocities).

    The construction of the dynamic window will be done by the following steps:

    First the search space will be construct by this expression:

    This expression describes the entire dynamic capabilities of the robot.

    Then, we reduce the search space to the only velocities that are in a breaking distance from an obstacle.

    The next expression contains the velocities that can be reached in the next time interval, according to the acceleration and velocities.

    The last step is to intersects all the expiration together to get the most reduced widow which construct from all the conditions above.

-   **Calculate the evaluation function** – In order to choose the best trajectory to move along, the algorithm calculates the evaluation function of each velocity couple (). The evaluation function is as follows:

    When is the direction to the target (higher when directly towards the target), is the distance from an obstacle (as close the obstacle, as lower the score), and is the robot’s velocity (score is higher when the velocity is higher).

Based on the above method, the algorithm can select the best trajectory to move along

For getting the safest and best path according to the evaluation function’s wights.

**The fusion algorithm concept**

As described above, the purpose of the algorithm is to combine between global and local path planning algorithms. The general idea of this concept can be seen in the following chart:

**![](media/9651c13da12b62829ee52803dd123dc6.emf)**

In the above chart, one can understand that the A\* algorithm serves as the base for the “dynamic window approach” algorithm, as it need to follow nodes along the A\* path. It is important to note that the A\* path is calculated single time in the beginning of the algorithm, while the Dynamic window algorithm constantly recalculates the best trajectory to move along.

In order to maintain the new path instead of moving towards the target, the researchers has modified the evaluation function:

As one can see, the new part of the equation has replaced with a part that programmed to follow the A\* path’s nodes. The results, gives a smoother path, as well as shortest path.

**![](media/f6a65f16214b21de36f0d588d518820b.png)**

**Simulation of 3D DWA and A\* using MATLAB**

In this part of the paper, I will describe in detail about my code and the functions I wrote to execute the simulation of the algorithm from the research paper. In addition to simulate the algorithm from the paper, I’ve enlarged it to operate in a 3-dimentaional space, Which I will cover in the following chapter.

**3D DWA environment** - For making the simulation to work in a 3D configuration space, there were two main modifications I needed to add to the regular DWA algorithm. I chose to treat this robot as a missile, so it is now has the ability to move only forward in the space according to the nose direction. The missile’s nose can move in yaw and pitch to make its way in the space. Therefore, the first modification, is adding another axis to the dynamic window. So instead of containing only the forward velocity and yaw velocity, the dynamic window now also contains the pitch velocity. So, it (the dynamic window) can be imaged as a 3-axis space.

The next modification in the algorithm, is to add to the evaluation function the heading towards the target in the YZ plane. So, the new evaluation function is now look like this:

**Configurations section** - The first section of the code (Configuration), contains all the data and definitions that are necessary for the algorithm to properly working. Along the things that located in this section are: Obstacle matrix (x,y,z of each obstacle), the robot’s dynamic constrains, the current state vector definition, time interval value, evaluation factors, etc.

**Main loop** – The main loop is the running center of the simulation. It has three main roles:

-   AstarPlan (Line 50) - It calls the A\* function and outputs a to column vector of the path coordinates x and y (Not in a loop, calls only once).

    ![Graphical user interface, text, application Description automatically generated](media/a6d1f4fbfeaa00922e49e3f1ac9343f1.png)

    At first, we create an “occupancy map” – a map with obstacles, using the function “setOccupancy” (built-in). Then I’ve built a “planner” which is an object contains the logic of the path planning (Using a built-in function “plannerAStarGrid”). Then we can call the function “plan” for planning the path from start point to an end point.

-   Function “f” (Actual movement) (Line 67) – calculates the new state vector according to the speed for the next time interval.

    ![](media/e630d5e55f6a66972bdbc02e96b5cf34.png)

    With the calculation above, the current state vector (red), gets added with the selected velocities vector (blue) that multiply by the time interval. This gives out a new vector which is the new state vector.

-   After those steps comes the animation plotting of the scene.
-   DynamicWindowAprroch (Line 65) - it calls the DWA function and outputs the selected velocities to the next movement

![](media/1bc5dec61c98ec5458efe2dcbd393ae2.png)

**Algorithm functions** – The next functions are all activated inside the “DynamicWindowAprroch” function in the main loop. These functions are the core functions of the algorithm.

![Graphical user interface, text, application, email Description automatically generated](media/5e72b81c79b32d542d610a3a17641a89.png)

-   “CalcDynamicWindow” – As it names suggests, this function calculates the dynamic window of the robot. It does that using the method described above (page 4 and 5). The function outputs a vector that contains the allowed speeds (min and max values of forward, horizontal, and vertical speeds).
-   “Evaluation” – this function, goes over every allowed speed in the dynamic window (with a preset resolution), and for every speed, it calls to the evaluation functions (dist, heading and heading in Z). the output is a vector which contains the values of each part of the evaluation function.
-   “NormalizeEval” – This is a simple function that normalize the values of each part of the evaluation function.
-   “GenerateTrajectory” – This function crates a trajectory towards the movement direction of the robot, that is later being plotted. This function is not working properly so I turned off the display of it.
-   “CalcHeadingEval” & “CalcDistFromZ” – are both functions that works in the same way for the same purpose but on different planes (one on XY and the other on YZ). Those functions calculate the degrees towards the target. If the robot moves directly towards the target (means the angle from the robot trajectory to the goal is 0) then it gets 180 which is the maximum score it can gets. The chosen direction is the one with the highest score.
-   “CalcDistEval” – Checks for the distance between the robot from the near obstacles. As far the robot from an obstacle, the higher the score (used for the evaluation function). The distance is simply calculated by a 3D “Normal” vector between the robot and position and the obstacle position with addition of the obstacle’s radius (can be preset in the configuration section).
-   “ClacBreakingDist” – A function that constantly calculates the break distance of the robot (the distance that would take the robot to get to 0 velocity). This distance than used to check whether the robot can stop before colliding with the next chosen speed by the evaluation function.

**Conclusions**

It can be seen in the result below, that the DWA algorithm serve as a good option in an unknown environment, and especially when it is needed to take in account the dynamical constrains of the robot (such as autonomous cars, Cruz missiles, etc.). When it gets combined with a global planning algorithm, as was suggested in the article and done in this simulation, it is easy to see that the robot does follow the shortest path of the A\* algorithm and gives the DWA another plane of autonomy.

![Chart, scatter chart Description automatically generated](media/86bae3111861c385a5c94b4d6dbeecab.jpeg)

1: The missile projectile hit to the target (Green)

**![Chart, line chart Description automatically generated](media/fa87d9f1b0affc4b28b6cfd1513713ed.jpeg)**

2: Top view of the missile as it follows the A\* path

An animated simulation can be found in the project’s google drive (attached link in the introduction section). \*\*  
\*\*
