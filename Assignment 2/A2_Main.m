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
axis([-1.7, 0.5, -1.5, 1, 0.01, 1.5]);
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

%% Farm Environment
% Tree Location Container
tree_position = [-0.4, 0.6, 0; 
                 -1.2, 0.6, 0;
                  0.4, 0.6, 0];

%% Initialise Farm

% Unsorted oranges crate
PlaceObject('fruit_crate_unsorted.ply',[-0.7,-0.35,0.01]);

% Ripe oranges crate
PlaceObject('fruit_crate_ripe.ply',[-1.3,-0.6,0.01]);

% Overripe oranges crate
PlaceObject('fruit_crate_over_ripe.ply',[-1.3,-1.0,0.01]);

% Trees
objFunc.PlaceManyObjects('treeOrange.ply',tree_position, false, 0, 0);

%% Tree 1 Oranges
% Tree 1 Orange Initial Locations
tree1_pos = [-0.4, 0.3, 0.5; 
             -0.6, 0.33, 0.4; 
             -0.5, 0.3, 0.6; 
             -0.6, 0.3, 0.51]; %3,4issue

% Store orange objects and vertices
tree1_obj = cell(1, size(tree1_pos, 1));
tree1_verts = cell(1, size(tree1_pos, 1));

% Tree 1 Orange picked off tree location
tree1_picked = tree1_pos;
tree1_picked(:, 2) = tree1_picked(:, 2) - 0.6; 


%% Tree 2 Oranges
% Tree 2 Orange Initial Locaitons
tree2_pos = [-1.1, 0.3, 0.35;
              -1.15, 0.3, 0.54;
              -0.9, 0.4, 0.5;
              -1, 0.4, 0.55];

% Store orange objects and vertices
tree2_obj = cell(1, size(tree2_pos, 1));
tree2_verts = cell(1, size(tree2_pos, 1));

% Tree 2 Orange picked off tree location
tree2_picked = tree2_pos;
tree2_picked(:, 2) = tree2_picked(:, 2) - 0.1;

%% Crate Positions
% Tree 1 Orange Crate Locations
tree1_crate_pos = [-0.55,-0.3,0.04; 
                   -0.65,-0.3,0.04; 
                   -0.75,-0.3,0.04; 
                   -0.85,-0.3,0.04];

% Tree 2 Orange Crate Locations
tree2_crate_pos = [-0.85,-0.37,0.04
                   -0.75,-0.37,0.04; 
                   -0.65,-0.37,0.04; 
                   -0.55,-0.37,0.04];

% Tree 1 Orange above crate
tree1_above_crate = tree1_crate_pos;
tree1_above_crate(:, 3) = tree1_above_crate(:, 3) + 0.8;

% Tree 2 Orange above crate
tree2_above_crate = tree2_crate_pos;
tree2_above_crate(:, 3) = tree2_above_crate(:, 3) + 0.8;

%% Grow Fruit

% Tree 1's oranges
[tree1_obj,tree1_verts] = objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);

% Tree 2's oranges
[tree2_obj,tree2_verts] = objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);

%% Generate LinearUR3
% Initialise LinearUR3
harvestBot = LinearUR3(transl(0,0,0.02));

%% Generate Franka Emika (Panda)
QA = Panda(transl(-0.7,-0.7,0.02));
QA.PlotAndColourPandaRobot();

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.


%% Harvest Tree 1
display(['Tree 1 Harvest: Beginning picking process.']);
o1_n = 0; % Orange harvest count

for x = 1:size(tree1_pos, 1)

    % Look for orange
    display(['Tree 1 Harvest: Look for orange ', num2str(x)]);
    robotFunctions.MoveRobot(harvestBot,[-0.5, 0.1, 0.5],50,0,false,0,1);
    
    % Move to initial orange location
    display(['Tree 1 Harvest: Go to orange ', num2str(x)]);
    robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2),tree1_pos(x,3)],50,0,false,0,1);

    % Gripper grasp orange

    % Move oranges away from tree
    display(['Tree 1 Harvest: Pick orange ', num2str(x), ' from tree.']);
    robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.2,tree1_pos(x,3)],50,tree1_obj{x},true,tree1_verts{x},2);

    % Move oranges above crate
    display(['Tree 1 Harvest: Place orange ', num2str(x), ' above crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.5],50,tree1_obj{x},true,tree1_verts{x},2);
% 
    % Place oranges in crate
    display(['Tree 1 Harvest: Place orange ', num2str(x), ' within the crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)],50,tree1_obj{x},true,tree1_verts{x},2);

    % Release gripper


    % Lift End Effector from crate
    display(['Tree 1 Harvest: Lift gripper ', num2str(x), ' from the crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.5],50,0,false,0,2);

    % Count orange picked.
    o1_n = o1_n + 1;
    display(['Tree 1 Harvest: The total number of oranges picked from tree 1 is ', num2str(x)]);
end

display(['Tree 1 Harvest: Completed.']);
display(['=====================================']);


%% Harvest Tree 2
display(['Tree 2 Harvest: Beginning picking process.']);
o2_n = 0; % Orange harvest count

for j = 1:size(tree2_pos, 1)

    % Look for orange
    display(['Tree 2 Harvest: Look for orange ', num2str(x)]);
    robotFunctions.MoveRobot(harvestBot,[-1.1, 0.1, 0.5],50,0,false,0,1);

    % Move to initial brick location
    display(['Tree 2 Harvest: Go to orange ', num2str(j)]);
    robotFunctions.MoveRobot(harvestBot,[tree2_pos(j,1),tree2_pos(j,2),tree2_pos(j,3)],50,0,false,0,1);

    % Gripper grasp orange

    % Move oranges away from tree
    display(['Tree 2 Harvest: Pick orange ', num2str(j), ' from tree.']);
    robotFunctions.MoveRobot(harvestBot,[tree2_picked(j,1),tree2_picked(j,2),tree2_picked(j,3)],50,tree2_obj{j},true,tree2_verts{j},1);

    % Move oranges to crate
    display(['Tree 2 Harvest: Place orange ', num2str(j), ' above crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)],50,tree2_obj{j},true,tree2_verts{j},2);

    % Place oranges in crate
    display(['Tree 2 Harvest: Place orange ', num2str(j), ' within the crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree2_crate_pos(j,1),tree2_crate_pos(j,2),tree2_crate_pos(j,3)],50,tree2_obj{j},true,tree2_verts{j},2);

    % Release gripper

    % Lift End Effector from crate
    display(['Tree 2 Harvest: Lift the gripper ', num2str(j), ' from the crate.']);
    robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)],50,0,false,0,2);

    % Count orange picked.
    o2_n = o2_n + 1;
    display(['Tree 2 Harvest: The total number of oranges picked from tree 2 is ', num2str(o2_n)]);
end

% Return Robot to beginning.
robotFunctions.MoveRobot(harvestBot,[-0.2, 0, 0.65],75,0,false,0,3);

%% End of program
display(['Harvesting completed.']);
sum_orange = x + j;
display(['Total oranges harvested: ', num2str(sum_orange)]);

%% Additional Notes
