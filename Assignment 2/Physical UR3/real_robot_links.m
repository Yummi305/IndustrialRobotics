close all;
clear all;
clc;

axis equal;
hold on;
r = UR3e();
r.model.teach();

% joints via teach
%    q1 q2 q3 q4 q5 q6
% 1. 0 -90 0 -90 0 0 starting
% 2. -126 -96 -90 3 83 -34 move towards
% 3. -126 -145 -15.4 -15.4 83 -34 move into orange
% 4. grip
% 5. -126 -96 -90 3 83 -34 move away from
% 6. 59 -108 -22 -16 83 -34 prep to place
% 7. 59 -108 -132 -28 83 -77 place
% 8. release gripper

%joints via real
% Note the default order of the joints is 3,2,1,4,5,6
%    q3 q2 q1 q4 q5 q6
% 1. 0 -90 0 -90 0 0 starting
% 2. -90 -96 -126 3 83 -34 move towards
% 3. -15.4 -145 -126 -15.4 83 -34 move into orange
% 4. grip
% 5. -90 -96 -126 3 83 -34 move away from
% 6. -22 -108 59 -16 83 -34 prep to place
% 7. -139 -108 59 -28 83 -77 place
% 8. release

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
