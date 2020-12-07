This document (`README_template.md`) provides a template for your **final** documentation (**NOT YOUR PROPOSAL**).

- Your final document should be named simply `README.md`.  
- I've left several comments below.  These should obviously be removed from your document.
- You may add additional sections as you see fit, but you should not **remove** any of the sections defined below.
- Any supplementary images should go in a directory called `Images`.  See [README.md](README.md) for more information about the required directory structure.
- Please keep in mind that the audience for this document should be students in the Fall 2019 section of this class.  (In other words, write this such that 3-month-younger you would've been able to utilize this document.)

---

# Quadruped Soccer

Project Name: catkin_ws/src/champ_config  
*For example, `followbot`, `wanderbot`, and `redball` are project names we've used in class.  When I install your code, I want to know where I'll find it in `~/catkin_ws/src/`*

Team Members:
- Caleb Terhune, calebter@buffalo.edu
- Shrinikethan Rajasekar, shrinike@buffalo.edu

---

## Project Description

*In this section, describe what your project does. This should be descriptive.  Someone from next year's class should be able to fully understand the aims and scope of your project. I highly recommend using pictures to help explain things.  Maybe even post a YouTube video showing your code in action.*

*NOTE:  This is not a proposal.  This is a final report describing your actual completed project.*

#### Aim of Project
The over all goal was to develop a fully autonomous quadruped that can score a goal by pushing a red ball between two goal posts.   

#### Scope of Project
We began by creating the gazebo world.  The gazebo world represents a kind of soccer field, but it has walls up as boundries.  The goal posts are marked in yellow at one end of the field.
*Insert: soccerField.png*
![ ](/home/ubuntu1804/Pictures/soccerField.png)

We then integrated code from the IE482/fall2018 repository on github.  This code was in the red ball module and the goal of the code was for a turtlebot to identify a red ball and move towards it.  The bot initially rotates until it sees the ball, this is the "finding the ball" stage.  Then once it gains sight of the ball it moves in on the ball until it has gotten approximately 1 meter away from the ball, and this is the "moving to ball" stage.  This is the window created by this initial code.
*Insert: find_move.png*
![](/home/ubuntu1804/Pictures/find_move.png) 

Then when the robot has gotten close enough to the ball it enters the "aiming kick" stage where it will revolve around the ball until it has lined up the ball between the goal posts. It does this by looking for two yellow masks on each half of the image.  When the ball is in between the center point of each goal post the quadruped stops revolving and moves into the "kicking the ball" stage.  Here are five images as the quadruped revolves around the ball until the kick is lined up.
*Insert: revolve_.png images*
![](/home/ubuntu1804/Pictures/revolve1.png) 
![](/home/ubuntu1804/Pictures/revolve2.png) 
![](/home/ubuntu1804/Pictures/revolve3.png) 
![](/home/ubuntu1804/Pictures/revolve4.png) 
![](/home/ubuntu1804/Pictures/revolve5.png) 

---
## Contributions

*In this subsection, I want to know what is new/unique/interesting about your project.*

We utilized a multitude of techniques that we learned throughout this year and while working on the project. 

#### Using masks to identify where colors/objects are in an image
We implemented the image from the camera on the quadruped into our code by detecting the red color of the ball and the yellow color of the goal.  The locations of the ball and goal are important for lining up the kick.  By creating two masks that identify each goal post and a third mask to identify the ball we create a way to line up the kick.  The quadruped waits until the ball is lined up between the goal posts and then it moves forward to kick the ball.

#### Twist commands to control our quadruped
The quadruped that we use was obtained from the chvmp/champ github (reference below).  These quadrupeds have built in capabilities to understand twist messages and move the legs accordingly.  The first commands are for the quadruped to find the ball, so it rotates until the red ball is in sight.  Second, it moves towards the ball until it is around 1 meter away.  Third, the quadruped revolves around the ball, keeping the ball centered in it's view, until the ball is lined up between the goal posts.  Finally, when the ball is in between the goal posts the quadruped moves forward to kick the ball towards the goal.

---

## Installation Instructions

### 1. Install Ubuntu 18.0.4.5 from the following link below:

[](https://releases.ubuntu.com/18.04/) 

#### 1.1 For Instructions to install Ubunutu please find the details from below link 

[](https://brb.nci.nih.gov/seqtools/installUbuntu.html) 

* - Note: -  Suggested to  use "dynamically allocated" hard drive rather that "fixed".

### 2. Installation ROS Melodic from the following link

[](http://wiki.ros.org/melodic/Installation/Ubuntu) 

### 3. Installation of ROS Packages for CHAMP Quadruped Controller

#### 3.1 Clone and install all dependencies:

```
sudo apt install -y python-rosdep
cd ~/catkin_ws/src
git clone --recursive https://github.com/chvmp/champ
git clone https://github.com/chvmp/champ_teleop
cd ..
rosdep install --from-paths src --ignore-src -r -y
 ```
 
#### 3.2 Build your workspace:

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
    
### 4. Walking demo in RVIZ:

#### 4.1  Run the base driver in Terminal 1:
        
```
source ~/catkin_ws/devel/setup.bash
roslaunch champ_config bringup.launch rviz:=true
```
- Note: - If have already runned the source bash.file command in terminal 1, then you don't have to run again.

#### 4.2  Run the teleop node in Terminal 2:

```
source ~/catkin_ws/devel/setup.bash
roslaunch champ_teleop teleop.launch
```
### 5. Walking demo in Gazebo environment:

#### 5.1 Run the Gazebo environment in Terminal 1:
```
source ~/catkin_ws/devel/setup.bash
roslaunch champ_config gazebo.launch
```
- Note: - If have already runned the source bash.file command in terminal 1, then you don't have to run again.

#### 5.2 Run the teleop node in Terminal 2:

```
source ~/catkin_ws/devel/setup.bash
roslaunch champ_teleop teleop.launch
```
Tested on the following operating system:
- [Ubuntu 16.04 (ROS Kinetic)] 
- [Ubuntu 18.04 (ROS Melodic)]

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




