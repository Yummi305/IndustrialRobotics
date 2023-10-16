robotFunctions = RobotFunctions();
QA = Panda(transl(0,0,0));
QA.PlotAndColourPandaRobot();

% For Panda

    
    q0 = deg2rad([-55,65.4,0,-25.3,0,259,0]); % Joint angle for pick-up of plates
    gripperLength + platestackHeight) * trotx(pi); % Transform of plates position at the stand

    q1 = myPanda.model.ikcon(platePosition{1},q0); % Inverse kinematics for joint angles 
    pandaQuinticMatrix = jtraj(myPanda.model.getpos,q1,steps); % Set quintic trajectory for the Panda
    
    % Sequence to move robot towards the plates initial positions
    for j = 1:steps
     pandaQ = pandaQuinticMatrix(j,:); % UR3 motion steps
     myPanda.model.animate(pandaQ)
     drawnow();
     pause(0.01)
    end

    q0 = deg2rad([-124,16.3,0,-31.9,0,252,0]); % Joint angle for drop off
   
    platePosition{2} = transl(plateCupboardPosition(i,1),plateCupboardPosition(i,2),plateCupboardPosition(i,3) + gripperLength + platestackHeight)... 
    * trotx(pi); % Transform of Final position of plates

    q1 = myPanda.model.ikcon(platePosition{2},q0); % Inverse kinematics for joint angles 
    pandaQuinticMatrix = jtraj(myPanda.model.getpos,q1,steps); % Set quintic trajectory
    
    % Sequence to pick-up, collect, and drop the plates with the robot
    for j = 1:steps
      pandaQ = pandaQuinticMatrix(j,:); % Panda motion steps
     myPanda.model.animate(pandaQ)
     drawnow();
     pause(0.01);
    end
end
