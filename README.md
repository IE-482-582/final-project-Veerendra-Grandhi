
---

# Optimal route for fetching shelves using robots in a warehouse

Project Name: multi_robot  

Team Members:
- Naga Veerendra Grandhi, nagaveer@buffalo.edu

Video Presentation link: https://youtu.be/howF9kEaziM

---

## Project Description
There is an ever increasing use of robotics in fulfillment centers such as Amazon. Fulfillment centers are densely packed with a large number of shelves to optimize the space. Transporter robots are used to lift and move shelves out of the way to retrieve a desired item. They aim to reduce average time for fetching an item by choosing the most optimal way for retrieval of a shelf in a fulfillment center with AS/RS by using these transporter robots. This project develops a prototype for such puzzle based goods storage and retrieval system by applying the concept of 15 puzzle. A set of 8 turtlebots are used in place of shelves while leaving 1 empty space within a 3 x 3 grid in the gazebo world. Our objective is to retireve a desired turtlebot out of the grid by using optimal path.

### Building Gazebo World
A four wall boundary is created in gazebo empty world. Inside multi_robot package, launch folder is created which has 3 launch files. The first one is "one_robot.launch", it has the description of turtlebot model and all the sensor connected to it. This file determines the topics to which our turtlebot subscribe's and publish's. The second one is "robots.launch", it calls "one_robot.launch" to launch the turtle bot. It creates 8 namespaces through which we can launch 8 turtlebots and place them in 8 different locations in the gazebo world. The third file "main.launch" calls "robots.launch" onto the four wall boundary world that has been created. Thereby placing 8 turtlebots inside a 3 x 3 grid boundary. The figure "multi_robot_grid_view" in images folder depicts the gazebo world for the project.

![logo](/images/multi_robot_grid_view.JPG)

### Python Code
Robots are indexed from 1 to 8 and we subscribe to /odom topic (Odometry) and publish to /cmd_vel_mux/input/teleop topic (Twist) of each turtlebot. Code is divided into 2 parts. First part deals with solving the puzzle and second part moves the turtlebots to reach the goal. The figure "turtlebot_index" in images folder can be used to choose the desired turtlebot to pull to the I/O point.

![logo2](/images/turtlebot_index.JPG)

#### Solve
First part is "Solver", which takes input argument from terminal while running python file. Input arugment is a number in between 1 to 8 that tells us which turtlebot needs to be brought to I/O point. While making the required moves to pull the desired turtlebots, an array is generated that contains [turtlebot index, move direction]. When the puzzle is solved, we have this array that lists exact order and direction in which the turtlebots should move to achieve our goal.

#### Move
Odometry topic initializes the start position of each turtlebot at (0,0,0) as the origin and sets the orientation in z to 0. We have the initial turtlebot locations from "robots.launch" file. By adding these co-ordinates to Odemetry pose for each corresponding turtlebot, we can get all the turtle bots into the same co-ordinate frame. Turtlebot moves are going to be one step at a time and restricted to four directions (front, back, right, left). The move method has 4 directional functions that are used for moving in each direction and these functions take robot index as input. The array developed in the solve method above is used as input for the move method.

### Contributions
*Although research has been done on optimizing storage systems, implementing a puzzle based solver by applying robotic technology can save time and human effort simultaneously in large warehouses. The code is completely parametric and can be applied for any number of robots. The path determined through this code is the optimal path for retrieval.*

---

## Installation Instructions

NOTE: These instructions are a little different from the textbook's.

1.  Create the Package:
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg multi_robot rospy gazebo_ros
    ```
    
2. Let's go ahead and create our `scripts`,`launch`and`world` directories:
    ```
    cd ~/catkin_ws/src/multi_robot
    mkdir scripts
    mkdir launch
    mkdir world
    ```
    	     
3. Get the source code from the course github site:
    ```
    cd ~/Downloads
    rm -rf final-project-Veerendra-Grandhi
    git clone https://github.com/IE-482-582/final-project-Veerendra-Grandhi.git
    ```
        
4. Copy the scripts, launch and world folders to our multi_robot workspace
    ```
    cd final-project-Veerendra-Grandhi/code/multi_robot
    cp scripts/* ~/catkin_ws/src/multi_robot/scripts/
    cp launch/* ~/catkin_ws/src/multi_robot/launch/
    cp world/* ~/catkin_ws/src/multi_robot/world/
    ```
    
5. Make our Python scripts executable
    ```
    cd ~/catkin_ws/src/multi_robot/scripts
    chmod +x *.py
    ```
    
6. Compile/make our package

    ```
    cd ~/catkin_ws
    catkin_make
    ```
---

## Running the Code

We'll need two (2) terminal windows.

1. Use a *launch* file to start roscore and Gazebo.

    ```
    cd ~/catkin_ws/src/multi_robot/launch
    roslaunch multi_robot main.launch
    ```

2. Run the puzzle_solver script:

    ```
    cd ~/catkin_ws/src/multi_robot/scripts
    rosrun multi_robot puzzle_solver.py 3
    ```
    *Enter any number in between 1 to 8 after `puzzle_solver.py`. This number is the turtlebot-index which tells our code to retrieve that particular robot. Refer to the figure "turtlebot_index" in images folder.*
---

## Measures of Success

<TABLE>
<TR>
	<TH>Measure of Success (from your PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>View turtlebots inside a boundary in Gazebo</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Demonstrate that a single turtle bot can be pulled outside the boundary in step by step manner</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Apply the 15 puzzle and retrieve the turtlebot using most optimal number of steps</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Implement code on a model that can have variable number of turtlebots</TD>
	<TD>90% (by varying number of robots, we need to change launch file and also need to publish and subscribe to topics related to new turtlebots)</TD>
</TR>
<TR>
	<TD>Have a classmate follow the steps in the README to successfully run the simulation without any help</TD>
	<TD>100%</TD>
</TR>
</TABLE>

---

## What did you learn from this project?

*What concepts from class do you now have a solid understanding of?  What new techniques did you learn?*
- Importing multiple robots into a custom created gazebo world.
- Using Odometry and Twist topic to move the robot for fixed length or fixed angle.

*What challenges did you face, and how did you overcome these?*
- The code is supposed to find empty space and move an "adjacent" turtlebot to make space for retrieving our target turtlebot. For this, we should publish to a robot by locating it in the co-ordinate frame. But publishing command is linked with a topic, ex: /robot5/cmd_vel_mux/teleop. We can't publish to any robot by just saying "Any robot at (x,y) to move".
- So, I created a 2D matrix of 3 x 3 with numbers as that of robot indexes. While solving this matrix, all the moves that led to retrieval of target are recorded into a new matrix. This matrix is named command matrix, it is list of elements of form (robot index, direction of move). Ex: (4, left). The command matrix is "fed" to move method which implented all these steps on turtlebots.

---

## Future Work

*If a student from next year's class wants to build upon your project, what would you suggest they do?  What suggestions do you have to help get them started (e.g., are there particular Websites they should check out?).*
- A storage system with two or more empty spaces can be implemented.
- A dual-load retrieval system which can pull out 2 loads together in the most optimal way can be implemented. Refer to "dual_load_retrieval" in images folder. Also, check out http://dspace.calstate.edu/bitstream/handle/10211.3/212826/Zaerpour201755.pdf?sequence=1.
![logo3](/images/dual_load_retrieval.JPG)

---

## References/Resources


- Kevin R. Gue, Byung Soo Kim, Puzzle‐based storage systems, Wiley InterScience (www.interscience.wiley.com), (2007), 556-567 https://onlinelibrary.wiley.com/doi/abs/10.1002/nav.20230: For solving the game of 15 puzzle.
- https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/: For launching multiple turtlebots.
- https://github.com/optimatorlab/turtlebotrace: For having all the required topics on turtlebot launch file.
- https://answers.ros.org/questions/: For queries regarding Odometry and Twist.
- https://stackoverflow.com/: For general python queries.
