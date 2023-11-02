%% Clear workspace before running script
clc;
clf;
clear;
% close all;

% Testing mode toggle
harvest_toggle = true;
QA_toggle = true;

% Create instance of Robot Functions Class in order to access functions
% robotFunctions = RobotFunctions();

% Create instance of Brick Functions Class in order to access functions
objFunc = ObjectFunctions();

% Create collision functions class to access functions
colFunc = CollisionFunctions();
% Setup environment
axis([-2.2, 1, -2, 1.2, 0.01, 1.3]); % original z = 1.5
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

% Farm Environment
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

% Unsorted Mandarins crate
PlaceObject('fruit_crate_unsorted.ply',[-0.7,-0.35,0.01]);

% Ripe Mandarins crate
PlaceObject('fruit_crate_ripe.ply',[-1.1,-0.6,0.01]);

% Overripe Mandarins crate
PlaceObject('fruit_crate_over_ripe.ply',[-1.1,-0.9,0.01]);

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
tree2_pos = [-0.95, 0.43, 0.47;
             -0.9, 0.4, 0.5;
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

%% Sorted Crate Positions
% Tree 1 Mandarin Sorted Crate Locations
tree1_sorted_crate_pos = [-1.2,-0.65,0.04; 
                          -1.1,-0.65,0.04; 
                          -1.0,-0.65,0.04; 
                          -1.1,-0.82,0.04];

% Tree 2 Mandarin Sorted Crate Locations
tree2_sorted_crate_pos = [-1.2,-0.52,0.04
                          -1.1,-0.52,0.04; 
                          -1.0,-0.52,0.04; 
                          -1.0,-0.82,0.04];

% % Test sorted crate orange placements
% objFunc.PlaceManyObjects('Mandarin_Ripe.ply',tree1_sorted_crate_pos, false, 0, 0);
% objFunc.PlaceManyObjects('Mandarin_Ripe.ply',tree2_sorted_crate_pos, false, 0, 0);

% Tree 1 Mandarin above sorted crate
tree1_above_sorted_crate = tree1_sorted_crate_pos;
tree1_above_sorted_crate(:, 3) = tree1_above_sorted_crate(:, 3) + 0.8;

% Tree 2 Mandarin above sorted crate
tree2_above_sorted_crate = tree2_sorted_crate_pos;
tree2_above_sorted_crate(:, 3) = tree2_above_sorted_crate(:, 3) + 0.8;

%% Grow Fruit
% If testing QA only, fast forward mandarin position to unsorted crate.
if (harvest_toggle == false && QA_toggle)
    % Tree 1's Mandarins
    [tree1_obj,tree1_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree1_crate_pos,tree1_obj,tree1_verts);
    % Tree 2's Mandarins
    [tree2_obj,tree2_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree2_crate_pos,tree2_obj,tree2_verts);
else
    % Else, spawn mandarin on tree.
    [tree1_obj,tree1_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree1_pos,tree1_obj,tree1_verts);
    % Tree 2's Mandarins
    [tree2_obj,tree2_verts] = objFunc.GrowMandarins('Mandarin_ripe.ply','Mandarin_OverRipe.ply',tree2_pos,tree2_obj,tree2_verts);
end

%% Generate LinearUR3
% Initialise LinearUR3
harvestBot = LinearUR3(transl(0,0,0.02));
hold on
harvestBot.model.getpos();

% Initialise Gripper on UR3 End Effector
% Initialise Gripper robots on UR3 end effector.
pos1 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); % Base position right gripper offset from UR3's end effector (0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
pos2 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); % Base position left gripper offset from UR3's end effector (-0.0127 is the ditance of the grip from the base cebtre and 0.0612 is the depth of the base)
g1 = GripRight(pos1); % initiate right gripper
g2 = GripLeft(pos2); % initial left gripper

% robotFunctions.GripperMove(g1,g2,1); % Close Gripper to operating distance for Mandarin (open close 10 degrees)

% Generate Franka Emika (Panda)
QA = Panda(transl(-0.6,-0.8,0.02));


% Franka Emika Gripper
% Initialise Gripper robots on Panda end effector.

%Using Robotiq 2f-145
pos3 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,-0.0127,0.05)*trotx(-pi/2)*trotz(pi);  % Base position right gripper offset from Panda's end effector 
pos4 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,0.0127,0.05)*trotx(-pi/2)*trotz(pi); % Base position left gripper offset from Panda's end effector 
g3 = GripRight(pos3); % initiate right gripper
g4 = GripLeft(pos4); % initial left gripper 


% robotFunctions.GripperMove(g3,g4,1); % Close Gripper to operating distance for Mandarin (open close 10 degrees)
clc
%% home robots
% robotFunctions.MoveRobot(QA,[-0.33,-0.8,1.145],40,0,false,0,0,g3,g4,2); %QA home pos
% 
% robotFunctions.MoveRobot(harvestBot, [-0.1942, 0, 0.7142],50,0,false,0,0,g1,g2,2);  %harvest home pos
% robotFunctions.MoveRobot(harvestBot, [-0.68, -.322, 0.36],50,0,false,0,2,g1,g2,2);  %harvest recording start pos


% robotFunctions.MoveRobot(harvestBot,[-0.5, -.2, 0.45],50,0,false,0,3,g1,g2,2); %harvest coll pos
% robot,position,steps,payload,holdingObject, vertices, endEffDirection,g_1,g_2,grip,
% robot2,position2,payload2,holdingObject2, vertices2, endEffDirection2,g_3,g_4,grip2)
% robotFunctions.MoveTwoRobots(harvestBot,[-0.68, -.2, 0.443],50,0,false,0,3,g1,g2,2, ...
%                             QA,[-0.679,-0.4,.68],0,false,0,2,g3,g4,2);

%% Franka self collision test
clc
% robotFunctions.MoveRobot(QA,[-1.47,-1.43,.55],40,0,false,0,3,g3,g4,2);
% robotFunctions.MoveRobot(QA, [-0.62, -1.67, .45], 40,0,false,0,2,g3,g4,2);
testmove(QA,[-.53,-.17,.45],80,0,false,0,2,g3,g4,2);
clc
testmove(QA, [-1.13, -.77, .25], 80,0,false,0,2,g3,g4,2);
clc
% robotFunctions.MoveRobot(QA,[-1.11,-1.4,.55],40,0,false,0,2,g3,g4,2);
testmove(QA, [.807, -.47, .35], 100,0,false,0,1,g3,g4,2);

%% LPI test

% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
% (planeNormal,pointOnPlane,point1OnLine,point2OnLine)
clf
clc
hold on
QA.PlotAndColourRobot()
g3 = GripRight(pos3); % initiate right gripper
g4 = GripLeft(pos4); % initial left gripper 
% robotFunctions.GripperMove(g3,g4,1);
harvestBot.PlotAndColourRobot()
g1 = GripRight(pos1); % initiate right gripper
g2 = GripLeft(pos2); % initial left gripper
% robotFunctions.GripperMove(g1,g2,1);
% [X,Y] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
% sizeMat = size(X);
% Z = repmat(0,sizeMat(1),sizeMat(2));
% oneSideOfCube_h = surf(X,Y,Z);
% plane = [X(:),Y(:),Z(:)];

harvestGrip1 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,0.127,0.2312);
harvestGrip2 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,-0.127,0.2312);
pointC = harvestGrip1(1:3,4);
pointD = harvestGrip2(1:3,4);

QAgrip1 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,-0.127,0.215);
QAgrip2 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,0.127,0.215);
pointA = QAgrip1(1:3,4);
pointB = QAgrip2(1:3,4);

X1 = pointA(1);
Y1 = pointA(2);
Z1 = pointA(3);
X2 = pointB(1);
Y2 = pointB(2);
Z2 = pointB(3);

X3 = pointC(1);
Y3 = pointC(2);
Z3 = pointC(3);
X4 = pointD(1);
Y4 = pointD(2);
Z4 = pointD(3);

plot3(linspace(X3,X4, 20), linspace(Y3,Y4, 20), linspace(Z3,Z4,20))

plot3(linspace(X1,X2, 20), linspace(Y1,Y2, 20), linspace(Z1,Z2,20))
planenormal = [0, 0, 1];
planepoint = [0,0,0];
check = LPIcheck(planenormal, planepoint, [X1,Y1,Z1], [X2,Y2,Z2])
% for i = 1:length(plane)
%     [intPoint, check] = LinePlaneIntersection([0,0,0], [0,0,0], [X1,Y1,Z1], [X2,Y2,Z2]);
%     intPoints{i} = intPoint(:)';
%     checks(i) = check;
% end
%%
% robotFunctions.MoveRobot(harvestBot, [])

harvestGrip1 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,0.127,0.2312);
harvestGrip2 = (harvestBot.model.fkineUTS(harvestBot.model.getpos()))*transl(0,-0.127,0.2312);
pointC = harvestGrip1(1:3,4);
pointD = harvestGrip2(1:3,4);

QAgrip1 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,-0.127,0.215);
QAgrip2 = (QA.model.fkineUTS(QA.model.getpos()))* transl(0,0.127,0.215);
pointA = QAgrip1(1:3,4);
pointB = QAgrip2(1:3,4);

LPIcheck([0,0,1], [0,0,0], [pointA(1), pointA(2), pointA(3)], [pointB(1), pointB(2), pointB(3)])
LPIcheck([0,0,1], [0,0,0], [pointC(1), pointC(2), pointC(3)], [pointD(1), pointD(2), pointD(3)])

%% Test ground collision detection

robotFunctions.MoveRobot(QA, [-.47, -.7, .75], 100,0,false,0,1,g3,g4,0);

testmove(QA, [.347, -.27, .1], 100,0,false,0,2,g3,g4,2);

%% test both robots movement will full collision
clc
robotFunctions.MoveTwoRobots(harvestBot,[0.18, -.2, 0.393],50,0,false,0,2,g1,g2,0, ...
                            QA,[-0.679, 0.4,.4],0,false,0,0,g3,g4,0, false);
robotFunctions.MoveTwoRobots(harvestBot,[-0.28, .62, 0.443],70,0,false,0,1,g1,g2,0, ...
                             QA,[-1.179,-0.86,.18],0,false,0,0,g3,g4,0, false);


%% test function
% test movement function with collision checking
function testmove(robot,position,steps,payload,holdingObject, vertices, endEffDirection,g_1,g_2,grip)
if (endEffDirection == 1)
    endMove = transl(position) * trotx(-pi/2); % To position end effector point in towards y axis in positive direction
elseif (endEffDirection == 2)
    endMove = transl(position) * trotx(pi); % To position end effector to point towards z axis in negative direction
elseif (endEffDirection == 3)
    endMove = transl(position) * trotx(pi/2); % To position end effector point in towards y axis in negative direction
else
    endMove = transl(position) * troty(-pi/2); % To position end effector to point towards x axis in negative direction
end
q0 = robot.model.getpos();
pose = robot.model.fkine(q0);
q1 = robot.model.ikcon(pose, q0);
q2 = robot.model.ikcon(endMove, q0);

s = lspb(0,1,steps);  % First, create the scalar function
qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
end

% Collision detection functions
collF = CollisionFunctions();

for i = 1:steps
    % Animation of Robot
    groundCheck = collF.collisionGroundLPI(robot); % variable to store if end effector will hit ground
    if collF.collisionCheckSelf(robot, qMatrix(i, :)) || groundCheck == 1  
        disp('(potential) Collision! Avoiding...')
        poseNow = robot.model.getpos();
        pointNow = robot.model.fkine(poseNow).T;
        pointNext = robot.model.fkine(qMatrix(i,:));
        pointAdj = pointNow;
        pointAdj = SE3(pointAdj(1:3, 4));
        invNext = SE3(inv(pointNext));
        pointAvoid = SE3(pointNow)*invNext*SE3(pointAdj); % aim directly away from collision
        poseAvoid = robot.model.ikcon(pointAvoid, poseNow);
        %         robot.model.animate(poseAvoid); % move the robot away from collision
        
        
        sideStep = 30;
        pointAvoid = SE3(pointNow)*invNext*SE3(pointAdj);
        q1 = robot.model.ikcon(pointNow,poseNow);
        q2 = robot.model.ikcon(pointAvoid, poseNow);
        s = lspb(0,1,sideStep);  % First, create the scalar function
        newqMatrix = nan(sideStep,length(robot.model.links));  % Create memory allocation for variables
        newqMatrix = (1-s)*q1 + s*q2;
        disp('new traj')
        for a = 1:sideStep
            robot.model.animate(newqMatrix(a,:));
            
%             robot.model.getpos();

            %         pause()
            pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
            pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612
            g_1.model.base = pos1;
            g_2.model.base = pos2;
            g_1.model.animate(g_1.model.getpos());
            g_2.model.animate(g_2.model.getpos());
            drawnow()
        end
        
        % regenerate new trajectory to endpoint
        q0 = robot.model.getpos();
        pose = robot.model.fkine(q0);
        q1 = robot.model.ikcon(pose, q0);
        
        % To adjust final position upward if input is too low (z is negative)
%       if groundCheck == 1
%             endMove = transl(0,0,.1)*endMove; % adjust endMove upwards if in ground
%         end
        q2 = robot.model.ikcon(endMove, q0);
        
        s = lspb(0,1,steps);  % First, create the scalar function
        qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
        qMatrix = (1-s)*q1 + s*q2;
%         for a = 1:steps
%             qMatrix(a,:) = (1-s(a))*q1 + s(a)*q2;
%         end
        disp('Collision avoided')
        
        for b = 1:i-1
            robot.model.animate(qMatrix(b,:));
            
%             robot.model.getpos();

            %         pause()
            pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
            pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612
            g_1.model.base = pos1;
            g_2.model.base = pos2;
            g_1.model.animate(g_1.model.getpos());
            g_2.model.animate(g_2.model.getpos());
            drawnow()
        end
        disp('Returning to main trajectory')
        
        
    end
%     qMatrix(i,:) - robot.model.getpos()
    
    robot.model.animate(qMatrix(i,:));
    
    % Gripper base transform for UR3.
    pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
    pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612


    g_1.model.base = pos1;
    g_2.model.base = pos2;
    g_1.model.animate(g_1.model.getpos());
    g_2.model.animate(g_2.model.getpos());

    %         if grip == 1 || grip == 2
    %             % Gripper open or close if necessary
    %             g_1.model.animate(qPath1(i,:));
    %             g_2.model.animate(qPath2(i,:));
    %         end

    % Apply transformation to objects vertices to visualise movement
    if holdingObject
        transMatrix = robot.model.fkine(qMatrix(i,:)).T; % create transformation matrix of current end effector position
        transMatrix = transMatrix*transl(0,0,0.2); % Manipulate translation matrix to offset object from end effector
        transfromedVert = [vertices,ones(size(vertices,1),1)] * transMatrix'; % transform vertices of object at origin position by transformation matrix
        set(payload,'Vertices',transfromedVert(:,1:3));
    end
    drawnow();
end
end
%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function check = LPIcheck(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end



