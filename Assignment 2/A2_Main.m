%% Clear workspace before running script
clc;
clf;
clear;
close all;

%% Testing mode toggle
harvest_toggle = false;
QA_toggle = true;

%% Create instance of Robot Functions Class in order to access functions
robotFunctions = RobotFunctions();

%% Create instance of Brick Functions Class in order to access functions
objFunc = ObjectFunctions();

%% Setup environment
axis([-2.2, 1, -2, 1.2, 0.01, 1.5]);
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

% Safety cones
PlaceObject('cone.ply',[-1.2,0,0.01]);
PlaceObject('cone.ply',[0.5,0,0.01]);
PlaceObject('cone.ply',[0.5,-1.4,0.01]);
PlaceObject('cone.ply',[-1.2,-1.4,0.01]);

PlaceObject('fireExtinguisherElevated.ply', [-1.8,0.65,0.01]);
PlaceObject('emergencyStopButton.PLY',[-2,-1.8,0.01]);

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
             -0.5, 0.3, 0.55; 
             -0.6, 0.3, 0.51;
             -0.6, 0.33, 0.4];

% Store orange objects and vertices
tree1_obj = cell(1, size(tree1_pos, 1));
tree1_verts = cell(1, size(tree1_pos, 1));

% Tree 1 Orange picked off tree location
tree1_picked = tree1_pos;
tree1_picked(:, 2) = tree1_picked(:, 2) - 0.6; 


%% Tree 2 Oranges
% Tree 2 Orange Initial Locaitons
tree2_pos = [-0.95, 0.43, 0.47;
             -0.9, 0.4, 0.5;
             -1, 0.38, 0.51;
             -1.1, 0.3, 0.39];

% Store orange objects and vertices
tree2_obj = cell(1, size(tree2_pos, 1));
tree2_verts = cell(1, size(tree2_pos, 1));

% Tree 2 Orange picked off tree location
tree2_picked = tree2_pos;
tree2_picked(:, 2) = tree2_picked(:, 2) - 0.1;

%% Unsorted Crate Positions
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

%% Sorted Crate Positions
% Tree 1 Orange Sorted Crate Locations
tree1_sorted_crate_pos = [-1.3,-0.6,0.04; 
                          -1.3,-0.6,0.04; 
                          -1.3,-0.6,0.04; 
                          -1.3,-1.0,0.04];

% Tree 2 Orange Sorted Crate Locations
tree2_sorted_crate_pos = [-1.3,-0.6,0.04
                          -1.3,-0.6,0.04; 
                          -1.3,-0.6,0.04; 
                          -1.3,-1.0,0.04];

% Tree 1 Orange above sorted crate
tree1_above_sorted_crate = tree1_sorted_crate_pos;
tree1_above_sorted_crate(:, 3) = tree1_above_sorted_crate(:, 3) + 0.8;

% Tree 2 Orange above sorted crate
tree2_above_sorted_crate = tree2_sorted_crate_pos;
tree2_above_sorted_crate(:, 3) = tree2_above_sorted_crate(:, 3) + 0.8;

%% Grow Fruit

% Tree 1's oranges
[tree1_obj,tree1_verts] = objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);

% Tree 2's oranges
[tree2_obj,tree2_verts] = objFunc.GrowOranges('Orange_ripe.ply','Orange_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);

%% Generate LinearUR3
% Initialise LinearUR3
harvestBot = LinearUR3(transl(0,0,0.02));

%% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on end effector.
pos1 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
pos2 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
g1 = GripRight(pos1); % initiate right gripper
g2 = GripLeft(pos2); % initial left gripper

%% Generate Franka Emika (Panda)
QA = Panda(transl(-0.7,-0.7,0.02));
QA.PlotAndColourPandaRobot();

%% Franka Emika Gripper
%  p_q0 = zeros (1, QA.model.n);
%  % Left Gripper
%  PandaLeft = PandaLeft(0); 
%  PandaLeft.model.base = PandaLeft.model.base * QA.model.fkine(p_q0) * transl(0,-0.155,0.03) * trotz(-pi/2) * trotx(pi/2); % Forward kinematics for end-effector base
%  PandaLeft.GetAndColourLeft()
%  % Right Gripper
%  PandaRight = PandaRight(0); % Finger 2 of Panda Gripper
%  PandaRight.model.base = PandaRight.model.base * QA.model.fkine(p_q0) * transl(0,-0.155,-0.03) * trotz(-pi/2) * trotx(pi/2) * trotx(pi); % Forward kinematics for end-effector base
%  PandaRight.GetAndColourRight()

pos3 = (QA.model.fkineUTS(QA.model.getpos())) * transl(0,-0.155,0.03) * trotz(-pi/2) * trotx(pi/2);
pos4 = (QA.model.fkineUTS(QA.model.getpos())) * transl(0,-0.155,-0.03) * trotz(-pi/2) * trotx(pi/2) * trotx(pi);
g3 = PandaRight(pos1); % initiate right gripper
g4 = PandaLeft(pos2); % initial left gripper
% PandaLeft.GetAndColourLeft();
% PandaRight.GetAndColourRight();

%% Toggle for testing harvest
if harvest_toggle 
    %% Harvest Tree 1
    display(['Tree 1 Harvest: Beginning picking process.']);
    o1_n = 0; % Orange harvest count
    
    for x = 1:size(tree1_pos, 1)
        % Look for orange
        display(['Tree 1 Harvest: Look for orange ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.45,tree1_pos(x,3)],50,0,false,0,1,g1,g2,0);
        
        % Move to initial orange location
        display(['Tree 1 Harvest: Go to orange ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.2,tree1_pos(x,3)],30,0,false,0,1,g1,g2,1);
    
    %     % Gripper grasp orange
    %     robotFunctions.GripperMove(g1,g2,1); % Not necessary as GripperMove
    %     has been integrated into MoveRobot
    
        % Move oranges away from tree
        display(['Tree 1 Harvest: Pick orange ', num2str(x), ' from tree.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_pos(x,1),tree1_pos(x,2)-0.45,tree1_pos(x,3)],30,tree1_obj{x},true,tree1_verts{x},1,g1,g2,0);
    
        % Move oranges above crate
        display(['Tree 1 Harvest: Place orange ', num2str(x), ' above crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.7],30,tree1_obj{x},true,tree1_verts{x},2,g1,g2,0);
     
        % Place oranges in crate
        display(['Tree 1 Harvest: Place orange ', num2str(x), ' within the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.2],30,tree1_obj{x},true,tree1_verts{x},2,g1,g2,0);
    
    %     % Release gripper
    %     robotFunctions.GripperMove(g1,g2,2); % Not necessary as GripperMove
    %     has been integrated into MoveRobot
    
        % Lift End Effector from crate
        display(['Tree 1 Harvest: Lift gripper ', num2str(x), ' from the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)+0.6],30,0,false,0,2,g1,g2,2);
    
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
        robotFunctions.MoveRobot(harvestBot,[tree2_pos(x,1),tree2_pos(x,2)-0.45,tree2_pos(x,3)],50,0,false,0,1,g1,g2,0);
    
        % Move to initial orange location
        display(['Tree 2 Harvest: Go to orange ', num2str(j)]);
        robotFunctions.MoveRobot(harvestBot,[tree2_pos(j,1),tree2_pos(j,2)-0.2,tree2_pos(j,3)],30,0,false,0,1,g1,g2,1);
    
    %     % Gripper grasp orange
    %     robotFunctions.GripperMove(g1,g2,1); % Not necessary as GripperMove
    %     has been integrated into MoveRobot
    
        % Move oranges away from tree
        display(['Tree 2 Harvest: Pick orange ', num2str(j), ' from tree.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_picked(j,1),tree2_picked(j,2)-0.45,tree2_picked(j,3)],30,tree2_obj{j},true,tree2_verts{j},1,g1,g2,0);
    
        % Move oranges to crate
        display(['Tree 2 Harvest: Place orange ', num2str(j), ' above crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)+0.7],30,tree2_obj{j},true,tree2_verts{j},2,g1,g2,0);
    
        % Place oranges in crate
        display(['Tree 2 Harvest: Place orange ', num2str(j), ' within the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_crate_pos(j,1),tree2_crate_pos(j,2),tree2_crate_pos(j,3)+0.2],30,tree2_obj{j},true,tree2_verts{j},2,g1,g2,0);
    
    %     % Release gripper
    %     robotFunctions.GripperMove(g1,g2,2); % Not necessary as GripperMove
    %     has been integrated into MoveRobot
    
        % Lift End Effector from crate
        display(['Tree 2 Harvest: Lift the gripper ', num2str(j), ' from the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)+0.7],30,0,false,0,2,g1,g2,2);
    
        % Count orange picked.
        o2_n = o2_n + 1;
        display(['Tree 2 Harvest: The total number of oranges picked from tree 2 is ', num2str(o2_n)]);
    end
    
    % Return Robot to beginning.
    robotFunctions.MoveRobot(harvestBot,[-0.2, 0, 0.65],50,0,false,0,3,g1,g2,1);
    
    display(['Tree 2 Harvest: Completed.']);
    display(['=====================================']);
end

%% Toggle for testing harvest
if QA_toggle
    %% Conduct Quality Control on set 1
    display(['Quality Control S1: Begin Filtering.']);
    
    for x = 1:size(tree1_sorted_crate_pos, 1)
        % Look for orange
        display(['Quality Control S1: Look for oranges to filter ', num2str(x)]);
        robotFunctions.MoveRobot(QA,[-0.7,-0.35,1.2],50,0,false,0,0,g3,g4,0);
        
        % Move to initial orange location
        display(['Quality Control S1: Go to orange ', num2str(x)]);
        robotFunctions.MoveRobot(QA,[tree1_crate_pos(x,1),tree1_crate_pos(x,2),tree1_crate_pos(x,3)],50,0,false,0,0,g3,g4,0);
    
        % Gripper grasp orange
    
        % Move oranges away from tree
        display(['Quality Control S1: Pick orange ', num2str(x), ' from tree.']);
        robotFunctions.MoveRobot(QA,[tree1_above_crate(x,1),tree1_above_crate(x,2)-0.2,tree1_above_crate(x,3)],50,0,false,0,2,g3,g4,0);
    
        % Move oranges above crate
        display(['Quality Control S1: Place orange ', num2str(x), ' above crate.']);
        robotFunctions.MoveRobot(QA,[tree1_above_sorted_crate(x,1),tree1_above_sorted_crate(x,2),tree1_above_sorted_crate(x,3)+0.5],50,0,false,0,2,g3,g4,0);
     
        % Place oranges in crate
        display(['Quality Control S1: Place orange ', num2str(x), ' within the crate.']);
        robotFunctions.MoveRobot(QA,[tree1_sorted_crate_pos(x,1),tree1_sorted_crate_pos(x,2),tree1_sorted_crate_pos(x,3)],50,0,false,0,2,g3,g4,0);
    
        % Release gripper
    
    
        % Lift End Effector from crate
        display(['Quality Control S1: Lift gripper ', num2str(x), ' from the crate.']);
        robotFunctions.MoveRobot(QA,[tree1_above_sorted_crate(x,1),tree1_above_sorted_crate(x,2),tree1_above_sorted_crate(x,3)+0.5],50,0,false,0,2,g3,g4,0);
    
        % Count orange picked.
        display(['Quality Control S1: The total number of oranges picked from tree 1 is ', num2str(x)]);
    end
    
    display(['Quality Control S1: Completed Filtering.']);
    display(['=====================================']);
    
    %% Conduct Quality Control on set 2
    display(['Quality Control S2: Begin Filtering.']);
    o2_n = 0; % Orange harvest count
    
    for j = 1:size(tree2_pos, 1)
    
        % Scan crate
        display(['Tree 2 Harvest: Look for orange ', num2str(x)]);
        robotFunctions.MoveRobot(harvestBot,[-1.1, 0.1, 0.5],50,0,false,0,0,g3,g4,0);
    
        % Move to initial brick location
        display(['Tree 2 Harvest: Go to orange ', num2str(j)]);
        robotFunctions.MoveRobot(harvestBot,[tree2_pos(j,1),tree2_pos(j,2),tree2_pos(j,3)],50,0,false,0,0,g3,g4,0);
    
        % Gripper grasp orange
    
        % Move oranges away from tree
        display(['Tree 2 Harvest: Pick orange ', num2str(j), ' from tree.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_picked(j,1),tree2_picked(j,2),tree2_picked(j,3)],50,0,false,0,0,g3,g4,0);
    
        % Move oranges to crate
        display(['Tree 2 Harvest: Place orange ', num2str(j), ' above crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)],50,0,false,0,0,g3,g4,0);
    
        % Place oranges in crate
        display(['Tree 2 Harvest: Place orange ', num2str(j), ' within the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_crate_pos(j,1),tree2_crate_pos(j,2),tree2_crate_pos(j,3)],50,0,false,0,0,g3,g4,0);
    
        % Release gripper
    
        % Lift End Effector from crate
        display(['Tree 2 Harvest: Lift the gripper ', num2str(j), ' from the crate.']);
        robotFunctions.MoveRobot(harvestBot,[tree2_above_crate(j,1),tree2_above_crate(j,2),tree2_above_crate(j,3)],50,0,false,0,0,g3,g4,0);
    
        % Count orange picked.
        o2_n = o2_n + 1;
        display(['Tree 2 Harvest: The total number of oranges picked from tree 2 is ', num2str(o2_n)]);
    end
    
    % Return Robot to beginning.
    robotFunctions.MoveRobot(harvestBot,[-0.2, 0, 0.65],75,0,false,0,0,g3,g4,0);
    
    display(['Quality Control S2: Completed Filtering.']);
    display(['=====================================']);
end
%% End of program
display(['Harvesting completed.']);
sum_orange = (x + j) - 2;
display(['Total number of oranges to be sold as fruit: ', num2str(sum_orange)]);
display(['Total number of oranges to be juiced: 2']);

%% Additional Notes
