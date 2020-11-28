
---

# Optimal route for fetching shelves using robots in a warehouse

Project Name: multi_robot  

Team Members:
- Naga Veerendra Grandhi, nagaveer@buffalo.edu

---

## Project Description
This project introduces and develops model for a physical goods storage system based on the 15‐puzzle, a classic children's game in which 15 numbered tiles slide within a 4 × 4 grid. The objective of the game is to arrange the tiles in numerical sequence, starting from a random arrangement. For our purposes, the tiles represent totes, pallets, or even containers that must be stored very densely, and the objective is to maneuver items to an input–output point for retrieval or processing. This project aims to reduce average time for fetching an item by choosing the most optimal way for retrieval of a shelf in a fulfillment center with AS/RS by using transporter robots. A set of 8 turtlebots are used in place of shelves within a 3 x 3 grid in the gazebo world.

### Building Gazebo World
A four wall boundary is created in gazebo empty world. Inside multi_robot package, launch folder is created which has 3 launch files. The first one is "one_robot.launch", it has the description of turtlebot model and all the sensor connected to it. This file determines the topics to which our turtlebot subscribe's and publish's. The second one is "robots.launch", it calls "one_robot.launch" to launch the turtle bot. It creates 8 namespaces through which we can launch 8 turtlebots and place them in 8 different locations in the gazebo world. The third file "main.launch" calls "robots.launch" onto the four wall boundary world that has been created. Thereby placing 8 turtlebots inside a 3 x 3 grid boundary. The figure "multi_robot_grid_view" in images folder depicts the gazebo world for the project.

### Python Code
Robots are indexed from 1 to 8 and we subscribe to /odom topic (Odometry) and publish to /cmd_vel_mux/input/teleop topic (Twist) of each turtlebot. Code is divided into 2 parts. First part deals with solving the puzzle and second part moves the turtlebots to reach the goal. The figure "turtlebot_index" in images folder can be used to choose the desired turtlebot to pull to the I/O point.

#### Solve
First part is "Solver", which takes input argument from terminal while running python file. Input arugment is a number in between 1 to 8 that tells us which turtlebot needs to be brought to I/O point. While making the required moves to pull the desired turtlebots, an array is generated that contains [turtlebot index, move direction]. When the puzzle is solved, we have this array that lists exact order and direction in which the turtlebots should move to achieve our goal.

#### Move
Odometry topic initializes the start position of each turtlebot at (0,0,0) as the origin and sets the orientation in z to 0. We have the initial turtlebot locations from "robots.launch" file. By adding these co-ordinates to Odemetry pose for each corresponding turtlebot, we can get all the turtle bots into the same co-ordinate frame. Turtlebot moves are going to be one step at a time and restricted to four directions (front, back, right, left). The move method has 4 directional functions that are used for moving in each direction and these functions take robot index as input. The array developed in the solve method above is used as input for the move method.

### Contributions
*Although research has been done on optimizing storage systems, implementing a puzzle based solver by applying robotic technology can save time and human effort simultaneously in large warehouses. The code is completely parametric and can be applied for any number of robots. The path determined through this code is the most optimal path for retireval.*

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

*Provide detailed step-by-step instructions to run your code.*

*NOTE 1:  At this point, the user should have already installed the necessary code.  This section should simply describe the steps for RUNNING your project.*  

*NOTE 2:  If you're generating mazes, for example, the task of GENERATING a new maze would go here.*

---

## Measures of Success

*You have already defined these measures of success (MoS) in your proposal, and updated them after your progress report.  The purpose of this section is to highlight how well you did.  Also, these MoS will be useful in assigning partial credit.*

*The MoS summary should be in table form.  A sample is provided below:*
<TABLE>
<TR>
	<TH>Measure of Success (from your PROPOSAL)</TH>
	<TH>Status (completion percentage)</TH>
</TR>
<TR>
	<TD>Install PR2 ROS Indigo Package</TD>
	<TD>100%</TD>
</TR>
<TR>
	<TD>Write brain reader software to move the robot</TD>
	<TD>25% (brain reader software detects brain waves, but does not translate to ROS commands.)</TD>
</TR>
</TABLE>

*NOTE 1:  I have your proposals...don't move the goal posts!*

*NOTE 2:  For activities less than 100% complete, you should differentiate between what you completed and what you were unable to complete. I suggest you add details in a bullet list below.* 


---

## What did you learn from this project?

*For example, what concepts from class do you now have a solid understanding of?  What new techniques did you learn?*

*Also, what challenges did you face, and how did you overcome these?  Be specific.*

---

## Future Work

*If a student from next year's class wants to build upon your project, what would you suggest they do?  What suggestions do you have to help get them started (e.g., are there particular Websites they should check out?).*

---

## References/Resources

*What resources did you use to help finish this project?*
- Include links to Websites.  Explain what this Website enabled you to accomplish.
- Include references to particular chapters/pages from the ROS book.  Why was each chapter necessary/helpful?



