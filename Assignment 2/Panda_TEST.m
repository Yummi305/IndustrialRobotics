close all;
clear all;
clc;

axis equal;
hold on;

robotFunctions = RobotFunctions();

% QA = Panda(transl(0,0,0));
% QA.model.teach();


r = UR3(transl(0,0,0));
r.model.teach();
