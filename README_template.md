
---

# Optimal route for fetching shelves using robots in a warehouse

Project Name: multi_robot  

Team Members:
- Naga Veerendra Grandhi, nagaveer@buffalo.edu

---

## Project Description
We introduce and develop model for a physical goods storage system based on the 15‐puzzle, a classic children's game in which 15 numbered tiles slide within a 4 × 4 grid. The objective of the game is to arrange the tiles in numerical sequence, starting from a random arrangement. For our purposes, the tiles represent totes, pallets, or even containers that must be stored very densely, and the objective is to maneuver items to an input–output point for retrieval or processing. This project aims to reduce average time for fetching an item by choosing the most optimal way for retrieval of a shelf in a fulfillment center with AS/RS by using transporter robots. A set of 8 turtle bots are used in place of shelves within a 3 x 3 grid in the gazebo world.

### Building Gazebo World
A four wall boundary is created in gazebo empty world. Inside multi_robot package, launch folder is created which has 3 launch files. The first one is "one_robot.launch", it has the description of turtlebot model and all the sensor connected to it. This file determines the topics to which our turtlebot subscribe's and publish's. The second one is "robots.launch", it calls "one_robot.launch" to launch the turtle bot. It creates 8 namespaces through which we can launch 8 turltlebots and place them in 8 different locations in the gazebo world. The third file "main.launch" calls "robots.launch" onto the four wall boundary world that has been created. Thereby placing 8 turtlebots inside a 3 x 3 grid boundary. The figure "multi_robot_grid_view" in images folder depicts the gazebo world for the project.
### Python Code
Robots are indexed from 1 to 8 and we subscribe to /odom topic (Odometry) and publish to /cmd_vel_mux/input/teleop topic (Twist) of each robot. Code is be divided into 2 parts of which first part deals with solving the puzzle and second part of the code moves the turtlebots to reach the goal. The figure "turtlebot_index" in images folder can be used to choose the desired turtlebot to pull to the I/O point.
#### Solve
First part is "Solver", which takes input argument from terminal while running python file. Input arugment is a number in between 1 to 8 that tells us which robot/shelf needs to be brought to I/O point. While making the required moves to pull the desired turtlebots, an array is generated that contains [turtlebot index, move direction]. When the puzzle is solved, we have this array that lists exact order and direction in which the turtlebots should move to achieve our goal.
#### Move
Odometry topic initializes the start position of each robot at (0,0,0) as the origin. We 

### Contributions

*In this subsection, I want to know what is new/unique/interesting about your project.*

---

## Installation Instructions

*In this section you should provide instructions for someone else to install all of the code necessary to execute your project.
Your target audience should be a student from the Fall 2020 class.
You may assume that the student has ROS Indigo installed on Ubuntu 14.04.*

List of Prerequisite Software:
- [software 1] 
- [software 2]
- [etc.]
*This is just a list, not installation instructions.  The idea is to provide a summary of the additional software/packages that need to be installed.  Instructions go below.*


*Now, provide detailed step-by-step instructions to install all necessary software for your project.*

*The expectation is that the user should only have to follow these steps one time.  For example, if your project requires generating Gazebo mazes, the task of INSTALLING the maze generation code should go in this section.*

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



