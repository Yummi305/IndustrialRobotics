%% Clear workspace before running script
clc;
clf;
clear;
close all;

%% Create instance of Robot Functions Class in order to access functions
robotFunctions = RobotFunctions();

%% Create instance of Brick Functions Class in order to access functions
objFunc = ObjectFunctions();

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

% Tree 1 Orange Initial Locations
tree1_pos = [-0.4, 0.3, 0.5; 
              -0.6, 0.33, 0.4; 
              -0.5, 0.35, 0.6; 
              -0.6, 0.35, 0.51];
tree1_obj = cell(1, size(tree1_pos, 1));
tree1_verts = cell(1, size(tree1_pos, 1));

% Tree 1 Orange Crate Locations
tree1_crate_pos = [-1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01];

% Tree 2 Orange Initial Locaitons
tree2_pos = [-1.1, 0.3, 0.35;
              -1.15, 0.3, 0.54;
              -0.9, 0.4, 0.5;
              -1, 0.4, 0.55;
              -1.25, 0.26, 0.45];
tree2_obj = cell(1, size(tree2_pos, 1));
tree2_verts = cell(1, size(tree2_pos, 1));

% Tree 2 Orange Crate Locaitons
tree2_crate_pos = [-1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01; 
                   -1.0,-0.5,0.01;
                   -1.0,-0.5,0.01];

%% Initialise objects

% Trees
objFunc.PlaceManyObjects('treeOrange.ply',tree_position, false, 0, 0);

% Tree 1's oranges
objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);

% Tree 2's oranges
objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);

%% Initialise equipment

% unsorted crate
PlaceObject('crate.ply',[-1.0,-0.5,0.01]);

% Ripe Oranges crate
PlaceObject('crate.ply',[-0.6,-0.5,0.01]);

% Overripe Oranges crate
PlaceObject('crate.ply',[-0.2,-0.5,0.01]);

%% Generate LinearUR3
% Initialise LinearUR3
harvestBot = LinearUR3(transl(0,0,0.01));

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.


%% Begin Picking
display(['Beginning planting process.']);

for x = 1:size(tree1_pos, 1)
    % Move to initial brick location
    robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2),tree1_pos(x,3)+0.1],50,0,false,0);

    % Pick up oranges

    % Move oranges away from tree

    % Move oranges to crate

    % Place oranges in crate

    % Release gripper

end

%% End of program
display(['Planting process completed.']);

%% Additional Notes
