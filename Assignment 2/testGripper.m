function testGripper()

clf
clc

bot = 1;

    switch bot
    case 1

        %GripLeft

% L1 = Link('d',0.06095,'a',0.02481/2,'alpha',pi/2,'offset',0,'qlim', [-pi,pi]);
% L2 = Link('d',0,'a',0.1,'alpha',0,'offset',deg2rad(40.07),'qlim', [0,deg2rad(49.93)]);
% L3 = Link('d',0,'a',0.0065,'alpha',0,'offset',deg2rad(49.93),'qlim', [deg2rad(-49.93),pi/8]);
% L4 = Link('d',0,'a',0.0605,'alpha',0,'offset',0,'qlim', [0,0]);

    L1 = Link('d',0,'a',0.1,'alpha',0,'offset',deg2rad(40.07),'qlim', [deg2rad(-40.07),0]);
    L2 = Link('d',0,'a',0.0065,'alpha',0,'offset',deg2rad(-40.07),'qlim', [0,deg2rad(40.07)]);
    L3 = Link('d',0,'a',0.0605,'alpha',0,'offset',0,'qlim', [0,0]);

robot = SerialLink([L1 L2 L3],'name','TestFingerBot1');

    case 2
 % Grip Right

    % L1 = Link('d',0.06095,'a',-0.02481/2,'alpha',-pi/2,'offset',0,'qlim', [-pi,pi]);
    % L2 = Link('d',0,'a',-0.1,'alpha',0,'offset',deg2rad(90-40.07),'qlim', [0,deg2rad(49.93)]);
    % L3 = Link('d',0,'a',-0.0065,'alpha',0,'offset',deg2rad(90-49.93),'qlim', [deg2rad(-49.93),pi/8]);
    % L4 = Link('d',0,'a',-0.0605,'alpha',0,'offset',0,'qlim', [0,0]);

    L1 = Link('d',0,'a',0.1,'alpha',0,'offset',deg2rad(-40.07),'qlim', [0,deg2rad(40.07)]);
    L2 = Link('d',0,'a',0.0065,'alpha',0,'offset',deg2rad(40.07),'qlim', [deg2rad(-40.07),0]);
    L3 = Link('d',0,'a',0.0605,'alpha',0,'offset',0,'qlim', [0,0]);


robot = SerialLink([L1 L2 L3],'name','TestFingerBot1');


    end

q = zeros(1,3); % Joints at zero position.
workspace = [-0.2 0.2 -0.2 0.2 -0.05 0.3];
scale = 0.5;
robot.plot(q,'workspace',workspace,'scale',scale);

robot.teach;

end


