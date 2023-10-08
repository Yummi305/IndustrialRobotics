function RobotModelMoveTest() % Move robot with gripper attached and move through example
        % Below model created using Cow Grip example
        
        %r.TestMoveJoints; % test joint moment within qlim
        g1 = Grip(transl(0,0,0)); % create grip1 class at origin for top jaw
        g2 = Grip2(transl(0,0,0)*trotz(pi)); %create grip2 at origin and rotate 180 degrees about to from bottom jaw
        
        disp('Loading robot again....')
        r = LinearUR3(transl(0,0,0.5));
        axis([-4, 2, -3, 2, 0.01, 4]);
        hold on;

        
        qPath = jtraj(r.model.qlim(:,1)',r.model.qlim(:,2)',200); % path between qlim of r
        
        for i = 1:length(qPath)
            r.model.animate(qPath(i,:));
            drawnow();
            g1.model.base = r.model.fkineUTS(r.model.getpos());
            g2.model.base = r.model.fkineUTS(r.model.getpos());
            g1.model.animate(g1.model.getpos());
            g2.model.animate(g2.model.getpos());
            drawnow();
            pause(0.02)
        end

    end