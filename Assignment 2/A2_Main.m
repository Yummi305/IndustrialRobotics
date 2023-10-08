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
axis([-2, 1, -1, 1, 0.01, 1.5]);
% axis equal;
hold on

% Read environment textures
 floor = imread ('animal_crossing__grass.jpg'); % Floor Image
%  wall = imread ('forest.jpg'); % Wall Image

% Floor
 surf([-6.5,-6.5;7,7],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',floor,'FaceColor','texturemap'); 

 hold on
% Walls 
%  surf([-4,2;-4,2],[2,2;2,2],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Back wall
%  surf([2,2;2,2],[-3,2;-3,2],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Side wall

%% Setup objects

% Tree Location Container
tree_position = [-0.4, 0.6, 0; 
                 -1.2, 0.6, 0];

% % Unripe Orange & Location Container
% unripe_initial = [-0.4, 0, 0.5; 
%                   -1.1, 0, 0.5];
% unripe_object = cell(1, size(unripe_initial, 1));
% unripe_verts = cell(1, size(unripe_initial, 1));

% Ripe Orange & Location Container
ripe_initial = [-0.4, 0.3, 0.5; 
                -0.6, 0.33, 0.4; 
                -0.5, 0.35, 0.6; 
                -1.1, 0.3, 0.35;
                -1.15, 0.3, 0.54
                -0.9, 0.4, 0.5
                ]; % i = 1 - 3, tree 1
                   % i = 4 - 6, tree 2
ripe_object = cell(1, size(ripe_initial, 1));
ripe_verts = cell(1, size(ripe_initial, 1));

% Overripe Orange & Location Container
overripe_initial = [-0.6, 0.35, 0.51; 
                   -1, 0.4, 0.55
                   -1.25, 0.26, 0.45
                   ]; % i = 1, tree 1
                      % i = 2 - 3, tree 2
overripe_object = cell(1, size(overripe_initial, 1));
overripe_verts = cell(1, size(overripe_initial, 1));


%% Initialise objects

% Trees
brickFunctions.PlaceManyObjects('treeOrange.ply',tree_position, false, 0, 0);

% Unripe Oranges
% brickFunctions.PlaceManyObjects('Orange_UnRipe.ply',unripe_initial, true, unripe_object, unripe_verts);

% Ripe Oranges
brickFunctions.PlaceManyObjects('Orange_ripe.ply',ripe_initial, true, ripe_object, ripe_verts);

% Overripe Oranges
brickFunctions.PlaceManyObjects('Orange_OverRipe.ply',overripe_initial, true, overripe_object, overripe_verts);

%% Initialise equipment

% unsorted crate
PlaceObject('crate.ply',[-1.0,-0.5,0.01]);

% Ripe Oranges crate
PlaceObject('crate.ply',[-0.6,-0.5,0.01]);

% Overripe Oranges crate
PlaceObject('crate.ply',[-0.2,-0.5,0.01]);

%% Generate LinearUR3
% Initialise LinearUR3
BrickBot = LinearUR3(transl(0,0,0.01));

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.


%% Begin Planting
display(['Beginning planting process.']);


%% End of program
display(['Planting process completed.']);

%% Additional Notes