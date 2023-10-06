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
axis([-3, 3, -3, 3, 0.01, 3]);
% axis equal;
hold on

%% hey dennis

% Read environment textures
 floor = imread ('animal_crossing__grass.jpg'); % Floor Image
%  wall = imread ('forest.jpg'); % Wall Image

% Floor
 surf([-6.5,-6.5;7,7],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',floor,'FaceColor','texturemap'); 

 hold on
% Walls 
%  surf([-4,2;-4,2],[2,2;2,2],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Back wall
%  surf([2,2;2,2],[-3,2;-3,2],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Side wall

%% Setup equipment

% PlaceObject('HalfSizedRedGreenBrick.ply',[1,-1,0]);
% PlaceObject('OrangeColoured.ply',[1.2,-1,0]);

%% Generate LinearUR3
% Initialise LinearUR3
BrickBot = LinearUR3(transl(0,0,0));
PlaceObject('TreeNormal.ply',[0,1.5,0]);

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.


%% Begin Planting
display(['Beginning planting process.']);


%% End of program
display(['Planting process completed.']);

%% Additional Notes
