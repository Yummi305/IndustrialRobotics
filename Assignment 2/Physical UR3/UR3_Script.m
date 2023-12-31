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

% q312456
% -2.5973 -0.3525 -1.0399 -0.3405 1.1689 0.0005 % picking pose
% q1:6
% -2.0399 -1.9081 -1.6591 -0.6422 1.5333 -0.4319 % picking pose
% -0.2974 -0.8499 -1.5208 -0.4375 1.2556 -1.6255 % picking pose
% -1.5024 -2.0525 -1.5782 -1.1746 1.5836 -0.7342 % downward placing pose
%  1.6528 -2.2856 -1.4590 -0.2505 1.4778 -0.5935 % picking prep pose for p6

% in q123456
% 1.5432 -1.5969 -1.7967 -1.0931 1.4739 -0.5848 % picking prep
% 1.0220 -1.4307 -1.8019 -0.6677 1.4741 -0.5839 % picking pose
% return to picking prep
% -1.2010 -1.9713 -1.5782 -1.5572 1.5840 1.1849 % placing prep
% -1.5025 -2.0525 -1.5782 -1.1747 1.5836 -0.7342 % placing pose
% 




%% tutorial source: 
% https://canvas.uts.edu.au/courses/27375/pages/instructions-for-using-a-real-ur3
% https://www.youtube.com/watch?v=3ugBHAXPOMg

%% Notes
% You MUST run this script per section (via Run Section). This is due to
% Matlab reading through the script quicker than the robot can handle. I.e.
% it will move onto the next section before the action is performed.

%% Connect

rosshutdown;

rosinit('192.168.27.1'); % If unsure, please ask a tutor

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

 

%% Position 1 starting

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2); % Pause to give time for a message to appear

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)]; % Update joint state to match real robot default.


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

nextJointState_123456 = deg2rad([-91.73 -90.1 -0.36 -174.91 91.24 132.73]); % initial grip facing down
% nextJointState_123456 = deg2rad([0.0 -1.5708 0.0 -1.5708 0.0 0.0]);
% nextJointState_123456 = deg2rad([-92 -89.39 -3.11 -100.12 266.71 178.01]);



endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);


sendGoal(client,goal);


%% Position 2  move toward orange

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

durationSeconds = 3; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

%  123:-0.2687807 -2.53073 -2.19911 goes into ground   |   321:-2.19911 -2.53073 -0.2687807
% nextJointState_123456 = [-2.5973 -0.3525 -1.0399 -0.3405 1.5145 0.0005];
% nextJointState_123456 = [-1.7967 -1.5969 1.5432 -1.0931 1.4739 -0.5848];
nextJointState_123456 = deg2rad([-92 -89.39 -3.11 -100.12 266.71 178.01]);


endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

 

%% Position 3 go to grab orange

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

durationSeconds = 3; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% to place orange/mandarin
%                        -1.5708 -1.67552 -2.19911
%    downward placement       [-1.5782 -2.0525 -1.5024 -1.1746 1.5836 -0.7342]
%    original placement       [-2.19911 -1.67552 -1.5708 0.0523599 1.44862 -0.593412]
% nextJointState_123456 = [-1.5782 -2.0525 -1.5024 -1.1746 1.5836 -0.7342];
% nextJointState_123456 = [-1.8260 -1.3205 1.0219 -0.8673 1.4747 -0.5839];
nextJointState_123456 = deg2rad([-92.18 -68.9 -2.59 -122.19 266.71 178.0]);

endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);


%% Position 4 Grip

[gripperPub, gripperMsg]= rospublisher('/onrobot_rg2/joint_position_controller/command');
gripperMsg.Data = 0.2; % 0.5 is open, -0.5 is closed
send(gripperPub, gripperMsg);


%% Position 5 move away from orange (Pick)

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

durationSeconds = 2; % This is how many seconds the movement will take

 

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     

      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% angled place pose
% nextJointState_123456 = [-1.5708 -1.67552 -2.19911 0.0523599 1.44862 -0.593412];

% move away back to grab prep pose
% nextJointState_123456 = [-1.7967 -1.5969 1.5432 -1.0931 1.4739 1.1848];
nextJointState_123456 = deg2rad([-92 -89.39 -3.11 -100.12 266.71 178.01]);


endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);

 

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

%% Position 6 move above placing zone

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

durationSeconds = 2.5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     
     
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% nextJointState_123456 = [-1.4593 -1.88496 1.02974 -0.279253 1.44862 -0.593412]; % pick up pose

% prep for placement pose
% nextJointState_123456 = [-1.5782 -1.9713 -1.2010 -1.5572 1.5840 1.1849];

% gripper
nextJointState_123456 = deg2rad([-91.85 -111.61 -36.1 -119.71 93.27 178]);


endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);


goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);


sendGoal(client,goal);


%% Position 7 place orange

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

durationSeconds = 3; % This is how many seconds the movement will take

 
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;

startJointSend.TimeFromStart = rosduration(0);     
      

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% nextJointState_123456 = [-2.42601 -1.88496 1.02974 -0.488692 1.44862 -1.3439];

% placement pose
% nextJointState_123456 = [-1.5781 -2.2595 -1.6502 -0.8197 1.5829 1.1845];

%gripper bot placement
% nextJointState_123456 = deg2rad([-92.44 -117.96 -56.05 -98.4 91.97 178.01]);
nextJointState_123456 = deg2rad([-88.57 -113.02 -63.48 -94.54 90.86 178]);


endJointSend.Positions = nextJointState_123456;

endJointSend.TimeFromStart = rosduration(durationSeconds);


goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

 

sendGoal(client,goal);

%% Position 8 Release Grip
[gripperPub, gripperMsg]= rospublisher('/onrobot_rg2/joint_position_controller/command');
gripperMsg.Data = 0.1; % 0.5 is open, -0.5 is closed
send(gripperPub, gripperMsg);


%% Return to picking Position 2 (Looking for oranges



