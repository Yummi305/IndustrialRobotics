classdef RobotFunctions
    % Class containing functions that facilitate robot movement.
    methods (Static)  
        %% Robot Movement
        function qEnd = MoveRobot(robot,position,steps,payload,holdingBrick, vertices)
            % move end effector to specified location and carry bricks if required
            % Obtain robots current position and desired position to form qMatrix
            q0 = robot.model.getpos();
            endMove = transl(position) * trotx(pi);
            pose = robot.model.fkine(q0);
            q1 = robot.model.ikcon(pose, q0);
            q2 = robot.model.ikcon(endMove, q0);
        
            % qMatrix calculation - smooth velocity and acceleration profiles
            % Method 1 Quintic Polynomial
%             qMatrix = jtraj(q1,q2,steps);

            % Method 2 Trapezoidal Velocity Profile - linear interpolation between points
            s = lspb(0,1,steps);  % First, create the scalar function
            qMatrix = nan(steps,length(robot.model.links));  % Create memory allocation for variables
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
            end
        
            % Execute the motion
                for i = 1:steps
                    robot.model.animate(qMatrix(i,:));
                    % Apply transformation to brick vertices to visualise movement
                    if holdingBrick
                        transMatrix = robot.model.fkine(qMatrix(i,:)).T;
                        transfromedVert = [vertices,ones(size(vertices,1),1)] * transMatrix';
                        set(payload,'Vertices',transfromedVert(:,1:3));
                    end
                    drawnow();
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
