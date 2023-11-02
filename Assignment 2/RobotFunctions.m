classdef RobotFunctions
    % Class containing functions that facilitate robot movement.

    properties (Access = public)
        
        %% eStop
        app
    end


    methods (Static)  
        
        function obj = RobotFunctions(app)
          obj.app = app; % use contructor to access app parameters from RobotFunctions
        end  
        
        %% Robot Movement
        function qEnd = MoveRobot(robot,position,steps,payload,holdingObject, vertices, endEffDirection,g_1,g_2,grip)
            % move end effector to specified location and carry bricks if required
            % Obtain robots current position and desired position to form qMatrix

            %% Set eStop bool to false
            StoreSwitchButtons.setgeteStop(false)
            StoreSwitchButtons.setgetManual(false)

            robotcount = 1;
              

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
            
            % Initalise collision functions for use
            collF = CollisionFunctions();
            
            % qMatrix calculation - smooth velocity and acceleration profiles
            % Method 1 Quintic Polynomial
%             qMatrix = jtraj(q1,q2,steps);

            % Method 2 Trapezoidal Velocity Profile - linear interpolation between points
            s = lspb(0,1,steps);  % First, create the scalar function
            qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end
        
            % If gripper is required to open or close, perform calculation.
            if grip == 1 || grip == 2 % grip == 0 means remain as is.

                % Open and close state set at +/- 10 degrees after initial
                % close (See GripperMove fucntion)
                leftQopen = [deg2rad(-20),deg2rad(20),0];
                rightQopen = [deg2rad(20),deg2rad(-20),0];
                leftQclosed = [deg2rad(-30),deg2rad(30),0];
                rightQclosed = [deg2rad(30),deg2rad(-30),0];

                if grip == 1
                    % Close Gripper
                    qPath1 = jtraj(rightQopen,rightQclosed,steps);
                    qPath2 = jtraj(leftQopen,leftQclosed,steps);
                elseif grip == 2
                    % Open Gripper
                    qPath1 = jtraj(rightQclosed,rightQopen,steps);
                    qPath2 = jtraj(leftQclosed,leftQopen,steps);
                end
            end

            % Execute the motion
                for i = 1:steps
                    
                    %Check estop at each step
                    [eStopValue, ~] = RobotFunctions.Check_eStop(StoreSwitchButtons.setgeteStop,StoreSwitchButtons.setgetManual);

                    if eStopValue == true
                    
                    Harvest_pos = robot.model.getpos();
                    Grip1_pos = g_1.model.getpos();
                    Grip2_pos = g_2.model.getpos();
                    Panda_pos = robot2.model.getpos();
                    Grip3_pos = g_3.model.getpos();
                    Grip4_pos = g_4.model.getpos();
                        

                    StopQs = [Harvest_pos Grip1_pos Grip2_pos, Panda_pos Grip3_pos Grip4_pos]; %Set stopQ container to store q values of each bot row 1 [robot, grip1, grip2] row 2 [robot2, grip3, grip 4]

                    RobotFunctions.eStop(robot,Harvest_pos,g_1,Grip1_pos,g_2,Grip2_pos,robot2,Panda_pos,g_3,Grip3_pos,g_4,Grip4_pos,StopQs);

                    disp ('Stop success, Return to loop')
                    else

                    end

                    % To check for collisions against self and ground
                    groundCheck = collF.collisionGroundLPI(robot);
                    selfCheck = collF.collisionCheckSelf(robot, qMatrix(i, :));

                    if selfCheck || groundCheck == 1
                        disp('(potential) Collision! Avoiding...')
                        poseNow = robot.model.getpos();
                        pointNow = robot.model.fkine(poseNow).T;
                        pointNext = robot.model.fkine(qMatrix(i,:));
                        pointAdj = pointNow;
                        pointAdj = SE3(pointAdj(1:3, 4));
                        invNext = SE3(inv(pointNext));
                        pointAvoid = SE3(pointNow)*invNext*SE3(pointAdj); % aim directly away from collision
                        poseAvoid = robot.model.ikcon(pointAvoid, poseNow);

                        sideStep = 30;
                        q1 = robot.model.ikcon(pointNow,poseNow);
                        q2 = robot.model.ikcon(pointAvoid, poseNow);
                        s = lspb(0,1,sideStep);  % First, create the scalar function
                        newqMatrix = nan(sideStep,length(robot.model.links));  % Create memory allocation for variables
                        newqMatrix = (1-s)*q1 + s*q2;
                        % disp('new traj')
                        for a = 1:sideStep
                            robot.model.animate(newqMatrix(a,:));
                            pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
                            pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612
                            g_1.model.base = pos1;
                            g_2.model.base = pos2;
                            g_1.model.animate(g_1.model.getpos());
                            g_2.model.animate(g_2.model.getpos());

                            if holdingObject
                                transMatrix = robot.model.fkine(newqMatrix(a,:)).T; % create transformation matrix of current end effector position
                                transMatrix = transMatrix*transl(0,0,0.2); % Manipulate translation matrix to offset object from end effector
                                transfromedVert = [vertices,ones(size(vertices,1),1)] * transMatrix'; % transform vertices of object at origin position by transformation matrix
                                set(payload,'Vertices',transfromedVert(:,1:3));
                            end
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

                        % match-up to current iteration in outer for loop
                        for b = 1:i-1
                            robot.model.animate(qMatrix(b,:));

                            pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
                            pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612
                            g_1.model.base = pos1;
                            g_2.model.base = pos2;
                            g_1.model.animate(g_1.model.getpos());
                            g_2.model.animate(g_2.model.getpos());

                            if holdingObject
                                transMatrix = robot.model.fkine(qMatrix(b,:)).T; % create transformation matrix of current end effector position
                                transMatrix = transMatrix*transl(0,0,0.2); % Manipulate translation matrix to offset object from end effector
                                transfromedVert = [vertices,ones(size(vertices,1),1)] * transMatrix'; % transform vertices of object at origin position by transformation matrix
                                set(payload,'Vertices',transfromedVert(:,1:3));
                            end
                            drawnow()
                        end
                    end

                    % Animation of Robot
                    robot.model.animate(qMatrix(i,:));

                    % Gripper base transform for UR3.
                    pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
                    pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612

                    
                    g_1.model.base = pos1; 
                    g_2.model.base = pos2; 
                    g_1.model.animate(g_1.model.getpos());
                    g_2.model.animate(g_2.model.getpos());

                    if grip == 1 || grip == 2
                        % Gripper open or close if necessary
                        g_1.model.animate(qPath1(i,:));
                        g_2.model.animate(qPath2(i,:));  
                    end

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
  

%% Dual Robot Movement
function MoveTwoRobots(robot,position,steps,payload,holdingObject, vertices, endEffDirection,g_1,g_2,grip,robot2,position2,payload2,holdingObject2, vertices2, endEffDirection2,g_3,g_4,grip2)
            % move end effector to specified location and carry bricks if required
            
            
            %% Set eStop bool to false
            StoreSwitchButtons.setgeteStop(false)
            StoreSwitchButtons.setgetManual(false)

            
            % Obtain robots current position and desired position to form qMatrix
            collisionCheck = CollisionFunctions();
            %% Robot1 end effector direction
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
            
            
            %% Robot2 end effector direction
            if (endEffDirection2 == 1)
                endMove2 = transl(position2) * trotx(-pi/2); % To position end effector point in towards y axis in positive direction
            elseif (endEffDirection2 == 2)
                endMove2 = transl(position2) * trotx(pi); % To position end effector to point towards z axis in negative direction
            elseif (endEffDirection2 == 3)
                endMove2 = transl(position2) * trotx(pi/2); % To position end effector point in towards y axis in negative direction
            else
                endMove2 = transl(position2) * troty(-pi/2); % To position end effector to point towards x axis in negative direction
            end

            q0_2 = robot2.model.getpos();
            pose_2 = robot2.model.fkine(q0_2);
            q1_2 = robot2.model.ikcon(pose_2, q0_2);
            q2_2 = robot2.model.ikcon(endMove2, q0_2);

          
            % qMatrix calculation - smooth velocity and acceleration profiles
            % Method 1 Quintic Polynomial
%             qMatrix = jtraj(q1,q2,steps);

            % Method 2 Trapezoidal Velocity Profile - linear interpolation between points

            s = lspb(0,1,steps);  % First, create the scalar function
            qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end

             %s2 = lspb(0,1,steps);  % First, create the scalar function %
             %not needed as common with both bots
             
            qMatrix2 = nan(steps,length(robot2.model.links));  % Create memory allocation for variables
            for i = 1:steps
                qMatrix2(i,:) = (1-s(i))*q1_2 + s(i)*q2_2;
            end
        
            % If gripper is required to open or close, perform calculation.
            if grip == 1 || grip == 2 || grip2 == 1 || grip2 == 2 % grip or grip2 == 0 means remain as is.

                % Open and close state set at +/- 10 degrees after initial
                % close (See GripperMove fucntion)
                leftQopen = [deg2rad(-20),deg2rad(20),0];
                rightQopen = [deg2rad(20),deg2rad(-20),0];
                leftQclosed = [deg2rad(-30),deg2rad(30),0];
                rightQclosed = [deg2rad(30),deg2rad(-30),0];

                if grip == 1 
                    % Close Gripper
                    qPath1 = jtraj(rightQopen,rightQclosed,steps);
                    qPath2 = jtraj(leftQopen,leftQclosed,steps);
                elseif grip == 2
                    % Open Gripper
                    qPath1 = jtraj(rightQclosed,rightQopen,steps);
                    qPath2 = jtraj(leftQclosed,leftQopen,steps);
                end

                if grip2 == 1 
                    % Close Gripper2
                    qPath3 = jtraj(rightQopen,rightQclosed,steps);
                    qPath4 = jtraj(leftQopen,leftQclosed,steps);

                elseif grip2 == 2
                    % Open Gripper2
                    qPath3 = jtraj(rightQclosed,rightQopen,steps);
                    qPath4 = jtraj(leftQclosed,leftQopen,steps);
                end

            end

            % Execute the motion
%                 for i = 1:steps
                sideSteps = steps - 20;
                i = 1;
                while i < steps
                    % Animation of Robot

                    %check for active estop
                   [eStopValue, ~] = RobotFunctions.Check_eStop(StoreSwitchButtons.setgeteStop,StoreSwitchButtons.setgetManual);

                    if eStopValue == true

                    Harvest_pos = robot.model.getpos();
                    Grip1_pos = g_1.model.getpos();
                    Grip2_pos = g_2.model.getpos();
                    Panda_pos = robot2.model.getpos();
                    Grip3_pos = g_3.model.getpos();
                    Grip4_pos = g_4.model.getpos();

                    StopQs = [Harvest_pos Grip1_pos Grip2_pos, Panda_pos Grip3_pos Grip4_pos]; %Set stopQ container to store q values of each bot row 1 [robot, grip1, grip2] row 2 [robot2, grip3, grip 4]

                    RobotFunctions.eStop(robot,Harvest_pos,g_1,Grip1_pos,g_2,Grip2_pos,robot2,Panda_pos,g_3,Grip3_pos,g_4,Grip4_pos,StopQs);

                    disp ('Stop success, Return to loop')
                    else

                    end

                    robot.model.animate(qMatrix(i,:));
                    robot2.model.animate(qMatrix2(i,:));
                    
                    % Gripper base transform for UR3.
                    pos1 = robot.model.fkineUTS(robot.model.getpos())*transl(0,-0.0127,0.0612)*troty(-pi/2);%z0.0612
                    pos2 = robot.model.fkineUTS(robot.model.getpos())*transl(0,0.0127,0.0612)*troty(-pi/2);%z0.0612
                    
                    point = (pos1(1:3, 4)+pos2(1:3, 4))/2;
                    robotsGripsCheck = collisionCheck.collisionCheckGrip(robot2, point);
                    % Gripper base transform for Panda.
                    pos3 = robot2.model.fkineUTS(robot2.model.getpos())*transl(0,-0.0127,0.05)*troty(-pi/2);%z0.0612
                    pos4 = robot2.model.fkineUTS(robot2.model.getpos())*transl(0,0.0127,0.05)*troty(-pi/2);%z0.0612
                    
                    groundCheck1 = collisionCheck.collisionGroundLPI(robot); % variable to store if end effector will hit ground
                    groundCheck2 = collisionCheck.collisionGroundLPI(robot2); % variable to store if end effector will hit ground
                    
                    selfCheck1 = collisionCheck.collisionCheckSelf(robot, qMatrix(i,:)); % variable to store if robot will hit self
                    selfCheck2 = collisionCheck.collisionCheckSelf(robot2, qMatrix2(i,:)); % variable to store if robot will hit self

                    if  robotsGripsCheck
                        % if robot2 tries to move into robot1
                        disp('Collision between robots Detected!');
                        % break %break loop and stop motion
                        if i <= 1 
                            poseA = robot.model.getpos();
                        else
                            poseA = qMatrix(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointA = robot.model.fkineUTS(poseA);
                        pointAadj = pointA(1:3, 4);
                        poseAnext = robot.model.fkineUTS(qMatrix(i,:));
                        avoidPointA = pointA*inv(poseAnext)*pointA;
                        avoidPoseA = robot.model.ikcon(avoidPointA, poseA); % point for robot to avoid collision

                        if i <= 1
                            poseB = robot2.model.getpos();
                        else
                            poseB = qMatrix2(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointB = robot2.model.fkineUTS(poseB);
                        pointBadj = pointB(1:3, 4);
                        poseBnext = robot2.model.fkineUTS(qMatrix2(i,:));
                        avoidPointB = pointB*inv(poseBnext)*pointB;
                        avoidPoseB = robot2.model.ikcon(avoidPointB, poseB); % point for robot to avoid collision

                        s1 = lspb(0,1,sideSteps);

                        firstqMatrix = (1-s1)*poseA + s1*avoidPoseA;
                        firstqMatrix2 = (1-s1)*poseB + s1*avoidPoseB;

                        % abs(steps -(i - sideSteps)) % if the collision occurs near the end, the actual new trajectory to endpoint wont need the full amount from 'steps'
                        s2 = lspb(0,1, steps - sideSteps); 
                        secondqMatrix = (1-s2)*avoidPoseA + s2*q2;
                        secondqMatrix2 = (1-s2)*avoidPoseB + s2*q2_2;

                        qMatrix = [firstqMatrix; secondqMatrix];
                        qMatrix2 = [firstqMatrix2; secondqMatrix2];
                        i = 1;

                    end

                    if groundCheck1 == 1 || selfCheck1
                        disp('Collision robot 1 Detected!');
                        % calculate new position to avoid collision
                        if i <= 1
                            poseA = robot.model.getpos();
                        else
                            poseA = qMatrix(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointA = robot.model.fkineUTS(poseA);
                        pointAadj = pointA(1:3, 4);
                        poseAnext = robot.model.fkineUTS(qMatrix(i,:));
                        avoidPointA = pointA*inv(poseAnext)*pointA;
                        avoidPoseA = robot.model.ikcon(avoidPointA, poseA); % point for robot to avoid collision

                        if i <= 1
                            poseB = robot2.model.getpos();
                        else
                            poseB = qMatrix2(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointB = robot2.model.fkine(poseB);
                        poseB = robot2.model.ikcon(pointB, poseB);
                        % remake new trajectory 
                        s1 = lspb(0,1,sideSteps);
                        firstqMatrix = (1-s1)*poseA + s1*avoidPoseA;
                        s2 = lspb(0,1, steps - sideSteps); 
                        secondqMatrix = (1-s2)*firstqMatrix(sideSteps, :) + s2*q2;
                        % robot2 likely unaffected, remake current trajectory
                        s3 = lspb(s(i),1, steps);
                        qMatrix2 = (1-s3)*poseB + s3*q2_2;

                        qMatrix = [firstqMatrix; secondqMatrix];
                        
                        i = 1; % reset interator

                    end

                    if groundCheck2 == 1
                        disp('Collision robot 2 Detected!');
                        % calculate new position to avoid collision
                        if i <= 1
                            poseB = robot2.model.getpos();
                        else
                            poseB = qMatrix2(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointB = robot2.model.fkineUTS(poseB);
                        pointBadj = pointB(1:3, 4);
                        poseBnext = robot2.model.fkineUTS(qMatrix2(i,:));
                        avoidPointB = pointB*inv(poseBnext)*pointB * transl(-.01,-.01, 0)*trotz(pi/10);
                        avoidPoseB = robot2.model.ikcon(avoidPointB, poseB); % point for robot to avoid collision
                        poseB = qMatrix2(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        if i <= 1 
                            poseA = robot.model.getpos();
                        else
                            poseA = qMatrix(i-1, :); % robot has moved at least once before being checked, i = 0 not possible
                        end
                        pointA = robot.model.fkine(poseA);
                        poseA = robot.model.ikcon(pointA, poseA);
                        % remake new trajectory 
                        s1 = lspb(0,1,sideSteps);
                        firstqMatrix = (1-s1)*poseB + s1*avoidPoseB;
                        s2 = lspb(0,1, steps - sideSteps); 
                        secondqMatrix = (1-s2)*firstqMatrix(sideSteps, :) + s2*q2_2;
                        qMatrix2 = [firstqMatrix; secondqMatrix];
                        qMatrix2(sideSteps-1:sideSteps+1, :)

                        % robot1 likely unaffected, remake current trajectory
                        s3 = lspb(s(i),1, steps);
                        qMatrix = (1-s3)*poseA + s3*q2;

                                                
                        i = 1; % reset interator

                    end
                    g_1.model.base = pos1; 
                    g_2.model.base = pos2; 
                    g_1.model.animate(g_1.model.getpos());
                    g_2.model.animate(g_2.model.getpos());

                    g_3.model.base = pos3; 
                    g_4.model.base = pos4; 
                    g_3.model.animate(g_3.model.getpos());
                    g_4.model.animate(g_4.model.getpos());
                    
                        

                    %% gripper

                    if grip == 1 || grip == 2
                        % Gripper open or close if necessary
                        g_1.model.animate(qPath1(i,:));
                        g_2.model.animate(qPath2(i,:));  
    
                    end

                    if grip2 == 1 || grip2 == 2 
                        % Gripper open or close if necessary  
                        g_3.model.animate(qPath3(i,:));
                        g_4.model.animate(qPath4(i,:)); 
                    end
                    
                    % Apply transformation to objects vertices to visualise movement
                    if holdingObject
                        transMatrix = robot.model.fkine(qMatrix(i,:)).T; % create transformation matrix of current end effector position
                        transMatrix = transMatrix*transl(0,0,0.2); % Manipulate translation matrix to offset object from end effector
                        transfromedVert = [vertices,ones(size(vertices,1),1)] * transMatrix'; % transform vertices of object at origin position by transformation matrix
                        set(payload,'Vertices',transfromedVert(:,1:3));
                    end

                    if holdingObject2
                        transMatrix = robot2.model.fkine(qMatrix2(i,:)).T; % create transformation matrix of current end effector position
                        transMatrix = transMatrix*transl(0,0,0.2); % Manipulate translation matrix to offset object from end effector
                        transfromedVert = [vertices2,ones(size(vertices2,1),1)] * transMatrix'; % transform vertices of object at origin position by transformation matrix
                        set(payload2,'Vertices',transfromedVert(:,1:3));
                    end

                    drawnow();
                    i = i + 1;
                end
            
            end

      function [ReturnPosition] = eStop(robot1,r1_currentpos,g_1,g1_currentpos,g_2,g2_currentpos,robot2, r2_currentpos,g_3,g3_currentpos,g_4,g4_currentpos,QpositionMat) % original gripper move function repurposed to inital movement only as other components are inside robot model move.
        
        ReturnPosition = QpositionMat;

        if StoreSwitchButtons.setgeteStop == true

             StopSteps = 10;

        qPathStop_R1 = jtraj(r1_currentpos,r1_currentpos,StopSteps);
        qPathStop_G1 = jtraj(g1_currentpos,g1_currentpos,StopSteps);
        qPathStop_G2 = jtraj(g2_currentpos,g2_currentpos,StopSteps);
        qPathStop_R2 = jtraj(r2_currentpos,r2_currentpos,StopSteps);
        qPathStop_G3 = jtraj(g3_currentpos,g3_currentpos,StopSteps);
        qPathStop_G4 = jtraj(g4_currentpos,g4_currentpos,StopSteps);

        while StoreSwitchButtons.setgeteStop == true
    
             for i = 1:StopSteps

                robot1.model.animate(qPathStop_R1(i,:));
                g_1.model.animate(qPathStop_G1(i,:));
                g_2.model.animate(qPathStop_G2(i,:)); 
                robot2.model.animate(qPathStop_R2(i,:));
                g_3.model.animate(qPathStop_G3(i,:));
                g_4.model.animate(qPathStop_G4(i,:));

                drawnow();

             end

             disp('stop loop run check') %debugging number of run times

             [eStopValue, ManualCheckValue] = RobotFunctions.Check_eStop(StoreSwitchButtons.setgeteStop,StoreSwitchButtons.setgetManual);

                %stoprequest = eStopValue;
                
                %% need to make edits here
                if (ManualCheckValue == true && eStopValue == false)

                    disp('Exit to Manual Overide')

                    break

                end
                
                if eStopValue == false
                   
                    disp('Resume Harvest/QA')
                    
                    break 
                end
                
        end

        elseif StoreSwitchButtons.setgeteStop == false

            disp('safe to continue') %temp debugging for estop

        end
    
  
        
        end
    
%% Check eStop's Value to see if changed in app.

function [eStopValue, ManualCheckValue] = Check_eStop(check_estopvalue,check_manual)


                eStopValue = check_estopvalue;
                ManualCheckValue = check_manual; 

                if (StoreSwitchButtons.setgeteStop == true)
                    
                ValueCheck = eStopValue;
                disp('STOP recognised')

                pause(2)

                elseif (StoreSwitchButtons.setgeteStop == false)
                    

                end


        end

        %% GripperMovement
        function GripperMove(g1,g2,goal) % original gripper move function repurposed to inital movement only as other components are inside robot model move.
        
        gsteps = 20; 
        
        Initial_leftQopen = zeros(1,3);
        Initial_rightQopen = zeros(1,3);
        Initial_leftQclosed = [deg2rad(-20),deg2rad(20),0];
        Initial_rightQclosed = [deg2rad(20),deg2rad(-20),0];
        
        if goal == 1

        % Close Gripper

        qPath1 = jtraj(Initial_rightQopen,Initial_rightQclosed,gsteps);
        qPath2 = jtraj(Initial_leftQopen,Initial_leftQclosed,gsteps);

        elseif goal == 2

        qPath1 = jtraj(Initial_rightQclosed,Initial_rightQopen,gsteps);
        qPath2 = jtraj(Initial_leftQclosed,Initial_leftQopen,gsteps);

        % Open Gripper

        end
        
        for i = 1:gsteps
                    g1.model.animate(qPath1(i,:));
                    g2.model.animate(qPath2(i,:));                
                    drawnow();
                    pause(0.001);
        end
        
        end

        %% Form Point Cloud
        function FormPointCloud(robot)
            stepRads = deg2rad(45);
            qlim = robot.model.qlim;
            prisStep = 0.79;
            prisStepQty = 3;
            qtyRevoluteJoint = 6; % Don't need to worry about joint 7
            pointCloudSize = prod(floor(1 + prisStepQty * ((2 * pi / stepRads)^qtyRevoluteJoint - 1))); % = 786430
            pointCloud = zeros(pointCloudSize,3);
            qContainer = zeros(pointCloudSize,7);
            counter = 2;
        
            % Go to all poses
            display(['Gathering q...']);
            tic
            for q1 = qlim(1,1):(prisStep/prisStepQty):qlim(1,2) % take step in non-radians due to prismatic (Between Limits)
                for q2 = qlim(2,1):stepRads:qlim(2,2)
                    for q3 = qlim(3,1):stepRads:qlim(3,2)
                        for q4 = qlim(4,1):stepRads:qlim(4,2)
                            for q5 = qlim(5,1):stepRads:qlim(5,2)
                                for q6 = qlim(6,1):stepRads:qlim(6,2)
                                    q7 = 0; % Don't need to worry about joint 7, just assume it=0
                                    q = [q1,q2,q3,q4,q5,q6,q7];
                                    qContainer(counter,:) = q;
                                    counter = counter + 1; 
                                    if mod(counter/pointCloudSize * 100,1) == 0
                                        display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudSize * 100),'% of poses']);
                                    end
                                end
                            end
                        end
                    end
                end
            end
            toc
            
            % Complete calculation and plot
            tic % begin counting time to do calculation
            display(['Beginning calculation for plot']);
            tr = robot.model.fkineUTS(qContainer); % Use fkineUTS instead of fkine so that .t is not needed, making it quicker.
            pointCloud = squeeze(tr(1:3,4,:))';
            toc % displays time elapsed
        
            % Plot the point cloud.
            hold on;
            plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            
            % Determine radius and volume
            % Find minimum x,y,z points.
            x_minimum = min(pointCloud(:,1));
            y_minimum = min(pointCloud(:,2));
            z_minimum = min(pointCloud(:,3));
             
            % Find maximum x,y,z points.
            x_minimum = max(pointCloud(:,1));
            y_minimum = max(pointCloud(:,2));
            z_minimum = max(pointCloud(:,3));
            
            % Calculate radius
            x_radius = (abs(x_minimum) + abs(x_minimum)) / 2;
            y_radius = (abs(y_minimum) + abs(y_minimum)) / 2;
            z_radius = (abs(z_minimum) + abs(z_minimum)) / 2;
            
            % Form matrix of radius
            radius_container =[x_radius,y_radius,z_radius];
        
            % Calculate maximum radius and volume 
            radius_maximum = max(radius_container);
            volume_maximum = (pi * (z_radius^2) * (2 * x_radius)); % Point cloud formed roughly resembles a cylinder. V = pi*r^2*h
        
            % Display results
            disp(['The maximum reach of the robot is the maximum radius in the x,y,z directions which is ', num2str(radius_maximum), ' in the z-direction.'])
            disp(['The radius in the x-direction is ', num2str(x_radius),'.'])
            disp(['The radius in the y-direction is ', num2str(y_radius),'.'])
            disp(['The approximate volume is ', num2str(volume_maximum),' which was calculated using the "volume of a cylinder" formula.'])
                
        end

    end
end
