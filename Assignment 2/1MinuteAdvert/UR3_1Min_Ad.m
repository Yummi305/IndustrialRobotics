%% Clear workspace before running script
clc;
clf;
clear;
close all;

%% Testing mode toggle
harvest_toggle = true;

%% Create instance of Robot Functions Class in order to access functions
robotFunctions = RobotFunctions();

%% Create instance of Brick Functions Class in order to access functions
objFunc = ObjectFunctions();

%% Setup environment
axis([-1.6,0.7, -0.7, 1.1, 0.01, 1]); % original
% axis([-1.6, 0.8, -0.45, 1.05, 0.01, 1]); % for orange picking clips
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

% Safety Equipment
% Guard rails to secure the zone from external hazards
PlaceObject('fence.ply',[-1.4,1,0.01]);
PlaceObject('fence.ply',[-0.15,1,0.01]);
PlaceObject('fence.ply',[1.1,1,0.01]);

% % Safety cones
% PlaceObject('cone.ply',[-1.2,0,0.01]);
% PlaceObject('cone.ply',[0.5,0,0.01]);
% PlaceObject('cone.ply',[0.5,-1.4,0.01]);
% PlaceObject('cone.ply',[-1.2,-1.4,0.01]);

% PlaceObject('fireExtinguisherElevated.ply', [-1.8,0.65,0.01]);
% PlaceObject('emergencyStopButton.PLY',[-2,-1.8,0.01]);

% Unsorted Mandarins crate
PlaceObject('fruit_crate_unsorted.ply',[-0.7,-0.35,0.01]);

% % Ripe Mandarins crate
% PlaceObject('fruit_crate_ripe.ply',[-1.1,-0.6,0.01]);
% 
% % Overripe Mandarins crate
% PlaceObject('fruit_crate_over_ripe.ply',[-1.1,-0.9,0.01]);

% Trees
objFunc.PlaceManyObjects('treeMandarin.ply',tree_position, false, 0, 0);

%% Tree 1 Mandarins
% Tree 1 Mandarin Initial Locations
tree1_pos = [-0.4, 0.3, 0.5; 
             -0.5, 0.3, 0.55; 
             -0.6, 0.3, 0.51;
             -0.6, 0.33, 0.4];

% Store Mandarin objects and vertices
tree1_obj = cell(1, size(tree1_pos, 1));
tree1_verts = cell(1, size(tree1_pos, 1));

% Tree 1 Mandarin picked off tree location
tree1_picked = tree1_pos;
tree1_picked(:, 2) = tree1_picked(:, 2) - 0.6; 


%% Tree 2 Mandarins
% Tree 2 Mandarin Initial Locaitons
tree2_pos = [-0.95, 0.38, 0.42;
             -0.9, 0.38, 0.48;
             -1, 0.38, 0.51;
             -1.1, 0.3, 0.39];

% Store Mandarin objects and vertices
tree2_obj = cell(1, size(tree2_pos, 1));
tree2_verts = cell(1, size(tree2_pos, 1));

% Tree 2 Mandarin picked off tree location
tree2_picked = tree2_pos;
tree2_picked(:, 2) = tree2_picked(:, 2) - 0.1;

%% Unsorted Crate Positions
% Tree 1 Mandarin Crate Locations
tree1_crate_pos = [-0.55,-0.3,0.04; 
                   -0.65,-0.3,0.04; 
                   -0.75,-0.3,0.04; 
                   -0.85,-0.3,0.04];

% Tree 2 Mandarin Crate Locations
tree2_crate_pos = [-0.85,-0.37,0.04
                   -0.75,-0.37,0.04; 
                   -0.65,-0.37,0.04; 
                   -0.55,-0.37,0.04];

% Tree 1 Mandarin above crate
tree1_above_crate = tree1_crate_pos;
tree1_above_crate(:, 3) = tree1_above_crate(:, 3) + 0.8;

% Tree 2 Mandarin above crate
tree2_above_crate = tree2_crate_pos;
tree2_above_crate(:, 3) = tree2_above_crate(:, 3) + 0.8;

%% Grow Fruit

% Tree 1's Mandarins
[tree1_obj,tree1_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);

% Tree 2's Mandarins
[tree2_obj,tree2_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);

%% Generate LinearUR3
% Initialise LinearUR3
harvestBot = LinearUR3(transl(0,0,0.02));

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on UR3 end effector.
pos1 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
pos2 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
g1 = GripRight(pos1); % initiate right gripper
g2 = GripLeft(pos2); % initial left gripper

robotFunctions.GripperMove(g1,g2,1); % Close Gripper to operating distance for Mandarin (open close 10 degrees)

%% Toggle for testing harvest
    %% Harvest Tree 1
    display(['Tree 1 Harvest: Beginning picking process.']);
    o1_n = 0; % Mandarin harvest count
    
    for x = 1:size(tree1_pos, 1)
        % Look for Mandarin
        display(['Tree 1 Harvest: Look for Mandarin ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.45,tree1_pos(x,3)],50,0,false,0,1,g1,g2,0);
        
        % Move to initial Mandarin location
        display(['Tree 1 Harvest: Go to Mandarin ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.2,tree1_pos(x,3)],30,0,false,0,1,g1,g2,1);
    
        % Move Mandarins away from tree
        display(['Tree 1 Harvest: Pick Mandarin ', num2str(x), ' from tree.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.45,tree1_pos(x,3)],30,tree1_obj{x},true,tree1_verts{x},1,g1,g2,0);
    
        % Move Mandarins above crate
        display(['Tree 1 Harvest: Place Mandarin ', num2str(x), ' above crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.7],30,tree1_obj{x},true,tree1_verts{x},3,g1,g2,0);
     
        % Place Mandarins in crate
        display(['Tree 1 Harvest: Place Mandarin ', num2str(x), ' within the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.2],30,tree1_obj{x},true,tree1_verts{x},2,g1,g2,0);

        % Lift End Effector from crate
        display(['Tree 1 Harvest: Lift gripper ', num2str(x), ' from the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.6],30,0,false,0,2,g1,g2,2);
    
       % Face left
        robotFunctions.MoveRobot(harvestBot,[-0.6,0,0.5],30,0,false,0,0,g1,g2,2);

        % Count Mandarin picked.
        o1_n = o1_n + 1;
        display(['Tree 1 Harvest: The total number of Mandarins picked from tree 1 is ', num2str(x)]);
    end
    
    display(['Tree 1 Harvest: Completed.']);
    display(['=====================================']);
    
    
    %% Harvest Tree 2
    display(['Tree 2 Harvest: Beginning picking process.']);
    o2_n = 0; % Mandarin harvest count
    
    for j = 1:size(tree2_pos, 1)
    
        % Look for Mandarin
        display(['Tree 2 Harvest: Look for Mandarin ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[tree2_pos(x,1),tree2_pos(x,2)-0.45,tree2_pos(x,3)],50,0,false,0,1,g1,g2,0);
    
        % Move to initial Mandarin location
        display(['Tree 2 Harvest: Go to Mandarin ', num2str(j)]);
        robotFunctions.MoveRobot(harvestBot,[tree2_pos(j,1),tree2_pos(j,2)-0.2,tree2_pos(j,3)],30,0,false,0,1,g1,g2,1);
    
        % Move Mandarins away from tree
        display(['Tree 2 Harvest: Pick Mandarin ', num2str(j), ' from tree.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_picked(j,1),tree2_picked(j,2)-0.45,tree2_picked(j,3)],30,tree2_obj{j},true,tree2_verts{j},1,g1,g2,0);
    
        % Move Mandarins to crate
        display(['Tree 2 Harvest: Place Mandarin ', num2str(j), ' above crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)+0.5],30,tree2_obj{j},true,tree2_verts{j},2,g1,g2,3);
    
        % Place Mandarins in crate
        display(['Tree 2 Harvest: Place Mandarin ', num2str(j), ' within the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_crate_pos(j,1),tree2_crate_pos(j,2),tree2_crate_pos(j,3)+0.2],30,tree2_obj{j},true,tree2_verts{j},2,g1,g2,0);
    
        % Lift End Effector from crate
        display(['Tree 2 Harvest: Lift the gripper ', num2str(j), ' from the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)+0.5],30,0,false,0,2,g1,g2,2);
    
        % Face left
        robotFunctions.MoveRobot(harvestBot,[-0.6,0,0.5],30,0,false,0,0,g1,g2,2);

        % Count Mandarin picked.
        o2_n = o2_n + 1;
        display(['Tree 2 Harvest: The total number of Mandarins picked from tree 2 is ', num2str(o2_n)]);
    end
    
    % Return Robot to beginning.
    robotFunctions.MoveRobot(harvestBot,[-0.2, 0, 0.65],50,0,false,0,3,g1,g2,1);
    
    display(['Tree 2 Harvest: Completed.']);
    display(['=====================================']);

%% End of program
display(['Harvesting completed.']);
sum_Mandarin = (x + j) - 2;
display(['Total number of Mandarins to be sold as fruit: ', num2str(sum_Mandarin)]);
display(['Total number of Mandarins to be juiced: 2']);

%% Additional Notes
