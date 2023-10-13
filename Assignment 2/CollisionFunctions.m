classdef CollisionFunctions
    properties 
    JointEllipse;
    end
    methods (Static)
        function jointellipsoid(robot) % generate ellipses around links at current pos.
            if exist('jointE', 'var')
                delete(jointE);
            end
            
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
            for j = 2:length(centreJoints)
                [ellipX, ellipY, ellipZ] = ellipsoid(centreJoints{j}(1), centreJoints{j}(2), centreJoints{j}(3),...
                                                     .05, .05, jointLinks(j).d/2) % generate the x, y, z points per link
                jointEllipse{j} = [ellipX(:), ellipY(:), ellipZ(:)].*jointTr(j).R';
                jointE = plot3(ellipX, ellipY, ellipZ); % generates the ellipse
            end
            
        end

        function outp = collisioncheck(robot)
            
        end
    end
end