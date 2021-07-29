# CSE310_Software_Portfolio

# Overview
Our goal is to learn how to use ROS to interact with the physical world and 
people. We want the arm to reach for a point designated by the user by touching
the point. It will also respond to voice commands (turn left or right, up or 
down, etc.). We will be learning Open CV and speech recognition to utilize the
commands mentioned earlier.

# Development Environment
The Raspberry Pi acts as the brain of the project, running Robot Operating 
System Noeticwith Python as the main language and a little C++. OpenCV (as used
with Google's Coral) is used to find objects. PocketSphinx translates speech to voice 
commands (for now, just 'object' to locate an object and throw it). Flite, a speech-to-text
program, is used to repond to the user. 

# Useful Websites
[Stack Overflow](https://stackoverflow.com/)  
[ROS Wiki](http://wiki.ros.org/Documentation)  
[YouTube](https://youtube.com)

# Story
This was a project to practice using OpenCV, the Robot Operating System, and Python. 

# Requirements
Flite  
PocketSphinx  
OpenCv  
ROS  
Coral - Run "git clone https://github.com/google-coral/examples-camera.git --depth 1" in terminal

# TODO
- [ ] Write setup script
- [ ] Switch from Coral to basic OpenCV to improve costs

## Authors:
### Jeff Marsh and Adam Amott
