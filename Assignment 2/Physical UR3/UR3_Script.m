%% Clean up
clear all;
close all;
clc;


%% Planned motion
%joints via real in radians
% Note the default order of the joints is 3,2,1,4,5,6
%    q3 q2 q1 q4 q5 q6
% 1. 0.0 -1.5708 0.0 -1.5708 0.0 0.0 starting
% 2. -1.5708 -1.67552 -2.19911 0.0523599 1.44862 -0.593412 move towards
% 3. -0.2687807 -2.53073 -2.19911 -0.2687807 1.44862 -0.593412 move into orange
% 4. grip
% 5. -1.5708 -1.67552 -2.19911 0.0523599 1.44862 -0.593412 move away from
% 6. -0.383972 -1.88496 1.02974 -0.279253 1.44862 -0.593412 prep to place
% 7. -2.42601 -1.88496 1.02974 -0.488692 1.44862 -1.3439 place
% 8. release

%% tutorial source: 
% https://canvas.uts.edu.au/courses/27375/pages/instructions-for-using-a-real-ur3
% https://www.youtube.com/watch?v=3ugBHAXPOMg

%% Notes
% You MUST run this script per section (via Run Section). This is due to
% Matlab reading through the script quicker than the robot can handle. I.e.
% it will move onto the next section before the action is performed.

%% Connect

% rosshutdown;

rosinit('192.168.27.1'); % If unsure, please ask a tutor

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

 

%% Position 1 starting

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

% jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [0.0 -1.5708 0.0 -1.5708 0.0 0.0];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

 

 

 

%% Position 2  move towards orange (distance away)

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [-1.5708 -1.67552 -2.19911 0.0523599 1.44862 -0.593412];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

 

%% Position 3 move into orange

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [-0.2687807 -2.53073 -2.19911 -0.2687807 1.44862 -0.593412];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);


%% Position 4 Grip

[gripperPub, gripperMsg]= rospublisher('/onrobot_rg2/joint_position_controller/command');
gripperMsg.Data = 0.5; % 0.5 is open, -0.5 is closed
send(gripperPub, gripperMsg);


%% Position 5 move away from orange

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [-1.5708 -1.67552 -2.19911 0.0523599 1.44862 -0.593412];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

%% Position 6 prep to place

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [-0.383972 -1.88496 1.02974 -0.279253 1.44862 -0.593412];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);


%% Position 7 place

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

 

jointStateSubscriber.LatestMessage

 

jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

 

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;

goal.Trajectory.Header.Seq = 1;

goal.Trajectory.Header.Stamp = rostime('Now','system');

goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.

durationSeconds = 5; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [-2.42601 -1.88496 1.02974 -0.488692 1.44862 -1.3439];

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

%% Position 8 Release Grip
[gripperPub, gripperMsg]= rospublisher('/onrobot_rg2/joint_position_controller/command');
gripperMsg.Data = 0.5; % 0.5 is open, -0.5 is closed
send(gripperPub, gripperMsg);




