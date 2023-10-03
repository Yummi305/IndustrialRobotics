%% Clear workspace before running script
clc;
clf;
clear;
close all;

%% Create instance of Robot Functions Class in order to access functions
robotFunctions = RobotFunctions();

%% Create instance of Brick Functions Class in order to access functions
brickFunctions = BrickFunctions();

%% Dimensions / Constants

%% Initialise object locations


%% Setup environment
clf; % Clear point cloud
clc; % Clear command window
axis([-4, 2, -3, 2, 0.01, 1.7]);
% axis equal;
hold on

% Read environment textures
 floor = imread ('slate_floor.jpg'); % Floor Image
 wall = imread ('Wall.jpg'); % Wall Image

% Floor
 surf([-4,-4;2,2],[-3,2;-3,2],[0.01,0.01;0.01,0.01],'CData',floor,'FaceColor','texturemap'); 

 hold on
% Walls 
 surf([-4,2;-4,2],[2,2;2,2],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Back wall
 surf([2,2;2,2],[-3,2;-3,2],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Side wall

%% Setup equipment


%% Generate LinearUR3
% Initialise LinearUR3
BrickBot = LinearUR3(transl(0,0,0));

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.


%% Begin Planting
display(['Beginning planting process.']);


%% End of program
display(['Planting process completed.']);

%% Additional Notes
