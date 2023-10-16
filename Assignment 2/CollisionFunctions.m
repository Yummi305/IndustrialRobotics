classdef CollisionFunctions
    properties
    JointEllipse;
    end
    methods (Static)
        function jointellipsoid(robot) % generate ellipses around links at current pos.
            if exist('JointEllipse')
                disp('prior surface found');
                delete(JointEllipse);
            end
            
            hold on
            % By taking transforms of each link:
            % T[j] = [R[j] p[j]; 0 1] 
            % where:
            % R[j] = rotations of joint
            % p[j] = joint location
            [Tr, jointTr] = robot.model.fkine(robot.model.getpos);
            jointLinks = robot.model.links;
            % then finding the centres of each link
            % cj = 1/2(p[j-1] + p[j])
            for i = 2:length(jointTr)-1

                centreJoints{i - 1} = .5*(jointTr(i - 1).t + jointTr(i).t);
                
            end
            if robot.plyFileNameStem == 'LinearUR3'
                rote = jointTr(2).tr2rpy;
                
                jointAdj = transl(centreJoints{2})*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                centreJoints{2} = jointAdj(1:3, 4);
            end
            
            % Spheroid can be made with centres and DH parameters 
            % 
            % A = R[1/rx^2 0 0;...
            %       0 1/ry^2 0;...
            %       0 0 1/rz^2]*R'
            % where:
            % A = defines shape of spheroid
            % R = rotation of spheroid
%             elip = jointTr.R*(eye(3))*jointTr.R';

            % then using equation: [x y z]*[1/rx^2 0 0;...
            %                               0 1/ry^2 0;...
            %                               0 0 1/rz^2]*[x; y; z]
            % to create ellipsoid
            % or use function
            for j = 1:length(centreJoints)

                [ellipX, ellipY, ellipZ] = ellipsoid(centreJoints{j}(1), centreJoints{j}(2), centreJoints{j}(3),...
                                                     .065, .065, jointLinks(j).d/1.5 - jointLinks(j).a/3); % generate the x, y, z points per link
                JointEllipse(j) = surf(ellipX, ellipY, ellipZ); % generates the ellipse
                Jrot = jointTr(j).tr2rpy;
                rotate(JointEllipse(j), [1 0 0], Jrot(1));
                rotate(JointEllipse(j), [0 1 0], Jrot(2));
                rotate(JointEllipse(j), [0 0 1], Jrot(3));
                %JointEllipse(j)
                
            end
           
            pause(3)
            delete(JointEllipse);
        end

        function outp = collisioncheck(robot)
            %
        end
    end
end