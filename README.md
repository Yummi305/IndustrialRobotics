# IndustrialRobotics
## ** Read Me **
Fruit Loops - Fruit Harvest and QA System

## Collaborators and Contribution
Dennis Nguyen | Yummi-305 | dennis.t.nguyen-1@student.uts.edu.au  
- GUI Design and implementation using App Designer including button logic  
- Environment Class and Environment Model implementation  
- Safety Model implementation  
- Panda Robot Model and Class
- BabyCow Class (Custom cow robot)
- Path planning and implementation of robot movement for one robot at a time  
- Robot simulation scenario implementation - manual vs autonomous, Harvest only, QA only, Harvest & QA  
- Manual controls / 'teach' functionality of robot and gripper for both robots  
- Recording simulation clips for 1-minute and 3-minte video  

Julien Wang | JW-355 | julien.wang@student.uts.edu.au  
- Implementation of Collision Detection and avoidance system  
- Usage of real UR3 robot  
- Recording simulation clips for 1-minute and 3-minte video  

Thomas Parish | tom-parish | thomas.parish-2@student.uts.edu.au  
- Gripper Model and Class  
- Implementation of robot movement for two robots at a time  
- Implementation of e-stop and resume  
- Usage of real UR3 robot  
- Vision, voice, and video editing of 1-minute and 3-minute video  

## 1-minute video link  
https://www.youtube.com/watch?v=wBfQ82aiZdo  

## 3-minute video link  
  
  
## Pre-requisites
You will need all of the relevant files to run this project.  
You will need to download the system app and relevant files.  
The app is called FruitLoop.mlapp, the other files included in this repository are required to run the system.  
git clone git@github.com:Yummi305/IndustrialRobotics.git  

You will need to download Peter Coorke's Robotics Toolbox modified by UTS.  
Access here: https://canvas.uts.edu.au/courses/27375/pages/subject-resources?module_item_id=1290469  

## How to run simulation
Run startup_rvc.m within the Robotics Toolbox.  
Run FruitLoop.mlapp within the Apps folder.  
This will open up the app and provide you with the option of running the system autonomously or manually.  
Options are enabled on start up. Actions that you cannot take (yet) will be disabled until appropriate.  

