classdef CollisionFunctions
    properties (Access = protected)
        ellipX; 
        ellipY;
        ellipZ;
        jointX;
        jointY;
        linkLeng;
        JointEllipse;
        centreJoints;
    end
    methods
        function self = CollisionFunctions() % generate ellipses around links at current pos.
%             self.updateEllips(robot);
%             if (strcmp(robot.plyFileNameStem, 'LinearUR3'))
%                 self.linkLeng = [0 .128 .2435 .131 .2132 .0853 .131];
%             end
%             if strcmp(robot.plyFileNameStem, 'ColouredPanda')
%                 self.linkLeng = [.333 0 .3160 0 .085 0 .385 0 0 .088 0];
%             end
            
%             if exist(self.JointEllipse)
%                 disp('prior surface found');
%                 delete(self.JointEllipse);
%             end
%             
%             hold on
%             % By taking transforms of each link:
%             % T[j] = [R[j] p[j]; 0 1] 
%             % where:
%             % R[j] = rotations of joint
%             % p[j] = joint location
%             tempAng = robot.model.getpos;
%             if length(tempAng) > 11
%                 tempAng = tempAng(1:11);
%             end
%             [Tr, jointTr] = robot.model.fkine(tempAng);
%             jointLinks = robot.model.links;
%             % then finding the centres of each link
%             % cj = 1/2(p[j-1] + p[j])
%             for i = 2:length(jointTr)-1
% 
%                 self.centreJoints{i - 1} = .5*(jointTr(i - 1).t + jointTr(i).t);
%                 
%             end
%             if strcmp(robot.plyFileNameStem, 'LinearUR3')
%                 rote = jointTr(2).tr2rpy;
%                 self.jointX = .06;
%                 self.jointY = .06;
%                 jointAdj = transl(self.centreJoints{2})*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
%                 self.centreJoints{2} = jointAdj(1:3, 4);
%             end
%             if strcmp(robot.plyFileNameStem,'ColouredPanda')
%                 self.jointX = .088;
%                 self.jointY = .088;
%             end
% 
%             for j = 1:length(self.centreJoints)
% 
%                 [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3),...
%                                                      self.jointX, self.jointY, jointLinks(j).d/1.5); % generate the x, y, z points per link
% %                 self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ); % generates the ellipse
%                 Jrot = jointTr(j).tr2rpy('deg');
% %                 rotate(self.JointEllipse(j), [0 0 1], (Jrot(3)));
% %                 rotate(self.JointEllipse(j), [0 1 0], (Jrot(2)));
% %                 rotate(self.JointEllipse(j), [1 0 0], (Jrot(1)));
% %                 
% %                 
% %                 self.JointEllipse(j);
%                 
%             end
           
            %pause(3)
            %delete(JointEllipse);
        end

        function visualiseEllips(self, robot, dur)
            hold on
            tempAng = robot.model.getpos;
%             if length(tempAng) > 11
%                 tempAng = tempAng(1:11);
%             end
            
            [Tr, jointTr] = robot.model.fkine(tempAng);
            
            jointLinks = robot.model.links;
            for i = 2:length(jointTr)
                centre = .5*(jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
                
            end
            if strcmp(robot.plyFileNameStem,'ColouredPanda')
                self.jointX = .058;
                self.jointY = .058;
%                 jointTr = jointTr([1:3, 5, 7, 9, 11]);
                self.linkLeng = [.15 0.001 .110 0.071 .095 0.131 .095 0.091 0.051 .041 0.001];
                rote = jointTr(7).tr2rpy;
                jointAdj = transl(self.centreJoints{7});
                jointAdj = jointAdj*trotz(rote(3))*troty(rote(2))*trotx(rote(1))*transl(0,0.1,0);
                self.centreJoints{7} = jointAdj(1:3, 4);
            end
            if strcmp(robot.plyFileNameStem,'LinearUR3')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.12 .098 .0435 .031 .032 .0853 .031];
                rote = jointTr(2).tr2rpy;
                jointAdj = transl(self.centreJoints{2});
                jointAdj = jointAdj*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                self.centreJoints{2} = jointAdj(1:3, 4);
            end
            
            for j = 1:length(self.centreJoints)
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3), self.jointX, self.jointY, self.linkLeng(j)); % generate points per link
                self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ); % generates the ellipse
                Jrot = rad2deg(jointTr(j).tr2rpy);
                  if j == 2 && strcmp(robot.plyFileNameStem,'LinearUR3')
                      Jrot(1) = Jrot(1)+90;
                  end
                rotate(self.JointEllipse(j), [0 0 1], Jrot(3), self.centreJoints{j});
                rotate(self.JointEllipse(j), [0 1 0], Jrot(2), self.centreJoints{j});
                rotate(self.JointEllipse(j), [1 0 0], Jrot(1), self.centreJoints{j});
               
            end
            pause(dur)
            delete(self.JointEllipse);
        end
        
        function updateEllips(self, robot)
            hold on
            tempAng = robot.model.getpos;
%             if length(tempAng) > 11
%                 tempAng = tempAng(1:11);
%             end
            
            [Tr, jointTr] = robot.model.fkine(tempAng);
            
            jointLinks = robot.model.links;
            for i = 2:length(jointTr)
                centre = .5*(jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
                
            end
            if strcmp(robot.plyFileNameStem,'ColouredPanda')
                self.jointX = .058;
                self.jointY = .058;
%                 jointTr = jointTr([1:3, 5, 7, 9, 11]);
                self.linkLeng = [.15 0.001 .110 0.071 .095 0.131 .095 0.091 0.051 .041 0.001];
                rote = jointTr(7).tr2rpy;
                jointAdj = transl(self.centreJoints{7});
                jointAdj = jointAdj*trotz(rote(3))*troty(rote(2))*trotx(rote(1))*transl(0,0.1,0);
                self.centreJoints{7} = jointAdj(1:3, 4);
            end
            if strcmp(robot.plyFileNameStem,'LinearUR3')
                self.jointX = .058;
                self.jointY = .058;
                self.linkLeng = [.12 .098 .0435 .031 .032 .0853 .031];
                rote = jointTr(2).tr2rpy;
                jointAdj = transl(self.centreJoints{2});
                jointAdj = jointAdj*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                self.centreJoints{2} = jointAdj(1:3, 4);
            end
            
            for j = 1:length(self.centreJoints)
                self.centreJoints{j};
                [self.ellipX, self.ellipY, self.ellipZ] = ellipsoid(self.centreJoints{j}(1), self.centreJoints{j}(2), self.centreJoints{j}(3), self.jointX, self.jointY, self.linkLeng(j)); % generate points per link
%                 self.JointEllipse(j) = surf(self.ellipX, self.ellipY, self.ellipZ); % generates the ellipse
%                 Jrot = rad2deg(jointTr(j).tr2rpy);
%                   if j == 2 && strcmp(robot.plyFileNameStem,'LinearUR3')
%                       Jrot(1) = Jrot(1)+90;
%                   end
%                 rotate(self.JointEllipse(j), [0 0 1], Jrot(3), self.centreJoints{j});
%                 rotate(self.JointEllipse(j), [0 1 0], Jrot(2), self.centreJoints{j});
%                 rotate(self.JointEllipse(j), [1 0 0], Jrot(1), self.centreJoints{j});
                
                
            end
%              pause();
%              delete(self.JointEllipse);
        end

        function outp = collisionCheckSelf(self, robot, Q) % check collision between end effector at point Q and links
            % outp = (point - centre)' * [XYZ Radii * I(3)] * (point - centre)
            tempAng = robot.model.getpos;
            [Tr, jointTr] = robot.model.fkine(tempAng);
            self.updateEllips(robot);
            if strcmp(robot.plyFileNameStem,'ColouredPanda')
                self.jointX = .088;
                self.jointY = .088;
                gripXY = 4*0.0127;
                gripZ = .05;
%                 jointTr = jointTr([1:3, 5, 7, 9, 11]);
                self.linkLeng = [.25 0.001 .160 0.071 .075 0.091 .095 0.091 0.051 .081 0.001];
            end
            for i = 2:length(jointTr)
                centre = .5*(jointTr(i - 1).t + jointTr(i).t);
                self.centreJoints{i - 1} = centre;
                
            end
            if strcmp(robot.plyFileNameStem,'LinearUR3')
                self.jointX = .058;
                self.jointY = .058;
                gripXY = 4*0.0127;
                gripZ = 0.0612;
                self.linkLeng = [0 .128 .0435 .131 .2132 .0853 .131];
                rote = jointTr(2).tr2rpy;
                jointAdj = transl(self.centreJoints{2});
                jointAdj = jointAdj*trotx(rote(1))*troty(rote(2))*trotz(rote(3))*transl(0, -0.12, 0);
                self.centreJoints{2} = jointAdj(1:3, 4);
                
            end
            point = robot.model.fkine(Q)*SE3(transl(0,0,.15));
            endTr = SE3(point);
            [gripEllX, gripEllY, gripEllZ] = ellipsoid(endTr.t(1), endTr.t(2), endTr.t(3), .015, .015, .015);
%             showElp = surf(gripEllX, gripEllY, gripEllZ);
%             Jrot = rad2deg(endTr.tr2rpy);
%             rotate(showElp, [0 0 1], Jrot(3), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             rotate(showElp, [0 1 0], Jrot(2), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             rotate(showElp, [1 0 0], Jrot(1), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             pause(.3)
%             delete(showElp)
            for i = 1:length(self.centreJoints)-2
                % outp <= 1 is a collision
                result = (point.t - self.centreJoints{i})' * ([self.jointX^-2, self.jointY^-2, self.linkLeng(i)^-2].*eye(3)) * (point.t - self.centreJoints{i});
                if result <= 1
                    
                    outp = true; % Return true if collision detected
                    return
                end
                
            end
            outp = false;
        end

        function outp = collisionCheckGrip(self, robot, P) % check collision between end effector and point P
            endAn = robot.model.getpos;
%             gripXY;
%             gripZ;
            if (strcmp(robot.plyFileNameStem, 'ColouredPanda'))
%                 endTr = endAn(1:11);
                gripXY = 9*0.0127;
                gripZ = .05;
            end
            if (strcmp(robot.plyFileNameStem, 'LinearUR3'))
                gripXY = 9*0.0127;
                gripZ = 0.0612;
            end
            endTr = SE3(robot.model.fkine(endAn).T*transl(0,0, gripZ+.18)); % convert end effector angles to transform and move to end of gripper

            % Visualises the ellipsoid then deletes it
%             [gripEllX, gripEllY, gripEllZ] = ellipsoid(endTr.t(1), endTr.t(2), endTr.t(3), gripXY, gripXY, gripZ);
%             showElp = surf(gripEllX, gripEllY, gripEllZ);
%             Jrot = rad2deg(endTr.tr2rpy);
%             rotate(showElp, [0 0 1], Jrot(3), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             rotate(showElp, [0 1 0], Jrot(2), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             rotate(showElp, [1 0 0], Jrot(1), [endTr.t(1), endTr.t(2), endTr.t(3)]);
%             pause(.3)
%             delete(showElp)
            if ~isrow(P)
                P = P';
            end
            
            place = (P - endTr.t');
            result = place * ([gripXY^-2, gripXY^-2, (5*gripZ)^-2].*eye(3)) * place';
            if result <= 1
                outp = true; % return true if collision detected
                return
            end
            outp = false;
        end

        function check = collisionGroundLPI(self, robot)
            planeNormal = [0,0,1];
            pointOnPlane = [0,0,0];
            if (strcmp(robot.plyFileNameStem, 'ColouredPanda'))
                QAgrip1 = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,-0.127,0.215);
                QAgrip2 = (robot.model.fkineUTS(robot.model.getpos()))* transl(0,0.127,0.215);
                point1 = [QAgrip1(1,4), QAgrip1(2,4), QAgrip1(3,4)];
                point2 = [QAgrip2(1,4), QAgrip2(2,4), QAgrip2(3,4)];
            end
            if (strcmp(robot.plyFileNameStem, 'LinearUR3'))
                harvestGrip1 = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,0.127,0.2312);
                harvestGrip2 = (robot.model.fkineUTS(robot.model.getpos()))*transl(0,-0.127,0.2312);
                point1 = [harvestGrip1(1,4), harvestGrip1(2,4), harvestGrip1(3,4)];
                point2 = [harvestGrip2(1,4), harvestGrip2(2,4), harvestGrip2(3,4)];
            end
            u = point2 - point1;
            w = point1 - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);

            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
%             intersectionPoint = point1OnLine + sI.*u; % not needed

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end

        function check = lightcurtainCheck(self, cow)
            curtainPoints = [-1.7,0.7; 
                            -1.7,-1.8; 
                             1,0.7;
                             1,-1.8];
            point1 = cow.model.base*SE3(transl(-.4, 0, 0));
            point1 = point1.T;
            point1 = point1(1:3, 4);
            point2 = cow.model.base*SE3(transl(.4, 0, 0));
            point2 = point2.T;
            point2 = point2(1:3, 4);
%             pointPlane;
            Normals = {[1, 0, 0],  [0, 1, 0]}; % normal to x, normal to y
            count = 0;
            

            for i = 1:length(curtainPoints)
                
                pointPlane = curtainPoints(i);
                for j = 1:length(Normals)
                    planeNormal = Normals{j};
                    u = point2 - point1;
                    w = point1 - pointPlane;
                    D = dot(planeNormal,u);
                    N = -dot(planeNormal,w);

                    outp= 0; %#ok<NASGU>
                    if abs(D) < 10^-7        % The segment is parallel to plane
                        if N == 0           % The segment lies in plane
                            outp= 2;
                            
                        else
                            outp = 0;       %no intersection
                            
                        end
                    end

                    %compute the intersection parameter
                    sI = N / D;
                    intersectionPoint = point1+ sI.*u;

                    if (sI < 0 || sI > 1)
                        outp= 3;          %The intersection point  lies outside the segment, so there is no intersection
                    else
                        outp=1;
                    end

                    % check for points inside the curtain
                    for a = 1:length(curtainPoints)
                        if point1(1) >= curtainPoints(1, 1) && point1(1) <= curtainPoints(3, 1)
                            count = count+1;
                        end
                        if point1(2) >= curtainPoints(2, 2) && point1(2) <= curtainPoints(3, 2)
                            count = count+1;
                        end
                        if point2(1) >= curtainPoints(1, 1) && point2(1) <= curtainPoints(3, 1)
                            count2 = count+1;
                        end
                        if point2(2) >= curtainPoints(2, 2) && point2(2) <= curtainPoints(3, 2)
                            count2 = count+1;
                        end
                        if count >= 2 || outp == 1 || count2 >= 2
                            check = true;
                            return
                        end
                    end
                end
            end
                
            

            % LPI check on the points
            
        end

        function qMat = remakeTraj(self, robot, sideSteps, steps, q2)
            % using basic RRT
            poseA = robot.model.getpos();
            planner = self.collisionCheckSelf(robot, poseA);
            qRand = poseA + .25*(2 * rand(1,length(robot.model.links)) - 1) * pi;
            if strcmp(robot.plyFileNameStem, 'LinearUR3')
                qRand(1) = poseA(1) + (rand(1)-.5)*.05;
            end
            trials = 0;
            while planner == true
                qRand = poseA + .25*(2 * rand(1,length(robot.model.links)) - 1) * pi;
                if strcmp(robot.plyFileNameStem, 'LinearUR3')
                    qRand(1) = poseA(1) + (rand(1)-.5)*.05;
                end
                planner = self.collisionCheckSelf(robot, qRand);
                trials = trials +1;
            end
            avoidPoseA = qRand;
            s1 = lspb(0,1,sideSteps);
            firstqMatrix = (1-s1)*poseA + s1*avoidPoseA;
            s2 = lspb(0,1, steps - sideSteps);
            secondqMatrix = (1-s2)*firstqMatrix(sideSteps, :) + s2*q2;
            qMat = [firstqMatrix; secondqMatrix];

        end
    end
end
    



% function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
% 
% algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
%               + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
%               + ((points(:,3)-centerPoint(3))/radii(3)).^2;
% end
