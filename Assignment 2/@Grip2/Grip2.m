classdef Grip2 < RobotBaseClass
    %% Create grip finger Robot

    properties(Access = public)              
        plyFileNameStem = 'Grip2';
    end
    
    methods
%% Define robot Function 
function self = Grip2(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();
            
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the Grip Finger Model 2
            
            link(1) = Link('theta',pi,'a',0,'alpha',-pi, 'qlim', [0 0.15],'offset',0.075);% prismatic link 1
            link(2)= Link('theta',0,'a',0.075,'alpha',-pi/2,'qlim', [0 0],'offset',0);% Static prismatic link 2 (grip face)
            link(3) = Link('d',0,'a',0.05,'alpha',pi, 'qlim', [deg2rad(-90) deg2rad(160)],'offset',0);% Revolute catch grip

            self.model = SerialLink(link,'name',self.name);

            q = zeros(1,3);
        

        end
    end
end