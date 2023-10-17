function RobotModelMoveTest() % Move robot with gripper attached and move through example
        % Below model created using Cow Grip example
        
        %r.TestMoveJoints; % test joint moment within qlim
        
        axis([-1.5, 1.5, -1.5, 1, -0.1, 1.6]);
        
        disp('Loading robot again....')
        r = LinearUR3(transl(0,0,0.5));
        
        hold on;

        pos1 = (r.model.fkineUTS(r.model.getpos()))*transl(0,0.0127,0.0612)*trotx(pi/2); %transl(0,-0.0127,0.0612) %*troty(-pi/2);
        pos2 = (r.model.fkineUTS(r.model.getpos()))*transl(0,-0.0127,0.0612)*trotx(pi/2); %*troty(-pi/2);

        g1 = GripRight(pos1); % create grip1 class at origin for top jaw.... transl(0,0,-0.0127)*trotz(pi/2)
        g2 = GripLeft(pos2); %create grip2 at origin and rotate 180 degrees about to from bottom jaw... transl(0,0,0.0127)*trotz(pi/2)

        steps = 200;
        
        %qPath = jtraj(r.model.qlim(:,1)',r.model.qlim(:,2)',200); % path between qlim of r

        qPath = jtraj(r.model.qlim(:,1)',r.model.qlim(:,2)',steps); % path between qlim of r
        
        for i = 1:steps %length(qPath)
            r.model.animate(qPath(i,:));
            pos1 = (r.model.fkineUTS(r.model.getpos()))*transl(0,-0.0127,0.0612)*troty(-pi/2);
            pos2 = (r.model.fkineUTS(r.model.getpos()))*transl(0,0.0127,0.0612)*troty(-pi/2);
            g1.model.base = pos1; 
            g2.model.base = pos2; 
            g1.model.animate(g1.model.getpos());
            g2.model.animate(g2.model.getpos());
            drawnow();
            pause(0.04)
        end

        pause(5)

    end