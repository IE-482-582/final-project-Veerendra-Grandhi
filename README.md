# Final Project

- This repo is where you will store **all** of your documents for the course project.
- Please see [PROPOSAL.md](PROPOSAL.md) for a template for your **proposal** (Due Oct. 25).

--- 

## IMPORTANT DATES:
- **Friday, Oct. 25, 10:00am** -- Be sure your repo has your final [PROPOSAL.md](PROPOSAL.md).  You will give your 5-minute proposal pitch in class that day, using the [PROPOSAL.md](PROPOSAL.md) file.
- **Monday, Nov. 25** -- Progress Report.  Each team will present the status of their project in class that day.
- **Friday, Dec. 6** -- Your **almost** final documentation, code, and presentation materials are due.  You'll be asked to give a brief presentation in class.  I'll give you feedback.
- **Exact Date to-be-determined** -- **Final** project presentations.  We'll either do these in class, or you'll produce a YouTube video.  We'll discuss in early December.
- **Friday, Dec. 13, Noon** -- Your complete project materials are due.


---

## Organizing your Repository
For consistency, please use the directory structure described below, where `projectname` should be replaced with the actual catkin_ws name of your project.
	
```
PROPOSAL.md
README.md
Images/	
code/projectname/	
	scripts/
	msg/
	srv/
	CMakeLists.txt
	package.xml
```		

- A sample README file [may be found here](README_template.md)
- `Images/` is a directory (folder) for storing the graphics for your README.
- `code/projectname/` is a directory for your ROS code.  Replace `projectname` with the name of your catkin package.
	- Within this directory you should have `CMakeLists.txt`, `package.xml`, a `scripts/` directory, most likely a `msg/` directory, and possibly a `srv/` directory (if your project uses services).  
- See `06_Followbot` for an example of the directory structure.


---

## Project Grading

Grades for the final project will be based on the following percentages and content:

- Proposal (10%)
- Progress Report (10%)
- Presentation (30%)
	- Did you prepare/rehearse for this presentation?
	- Is the README neatly formatted?
	- Is the README (nearly) complete?
	- Was the code submitted/organized properly?  Are filenames correct?  Code in the proper directories/subdirectories?
	- Are the installation instructions complete?
	- Are the instructions for running the code complete?
	- Were you able to answer technical questions about your project?
	- How well were you able to demonstrate the actual implementation?  Note: You have until Friday, Dec. 13 to finalize the project.
- Final Documentation and Code (30%)
	- Did you address issues from the presentation feedback?
	- How did you do on the "measures of success"?
- Can the Instructor Run your Code? (20%)
	- Can the instructor successfully install the prereqs?
	- Can the instructor successfully run the code?  (I highly recommend that you find someone to test this for you)
	- Does the code do what it's supposed to?
				