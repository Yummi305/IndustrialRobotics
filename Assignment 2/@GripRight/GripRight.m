classdef GripRight < RobotBaseClass
    %% Create Grip finger Robot for Right side of Robitiq 2F-140

    properties(Access = public)              
        plyFileNameStem = 'GripRight';
    end
    
    methods
%% Define robot Function 
function self = GripRight(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % DH parameters   

            link(1) = Link('d',0.06095,'a',0.02481/2,'alpha',pi/2,'offset',0,'qlim', [-pi,pi]);
            link(2) = Link('d',0,'a',0.1,'alpha',0,'offset',deg2rad(40.07),'qlim', [0,deg2rad(49.93)]);
            link(3) = Link('d',0,'a',0.0065,'alpha',0,'offset',deg2rad(49.93),'qlim', [deg2rad(-49.93),pi/8]);
            link(4) = Link('d',0,'a',0.0605,'alpha',0,'offset',0,'qlim', [0,0]);

            self.model = SerialLink(link,'name',self.name);

            q = zeros(1,4);
        

        end
    end
end