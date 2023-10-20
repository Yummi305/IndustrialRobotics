classdef Panda < RobotBaseClass
    properties(Access = public)    
        plyFileNameStem = 'Panda';
    end
    
    methods 
%% Define robot Function 
        function self = Panda(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr;            
            self.PlotAndColourRobot();         
        end
%% Create the robot model
        function CreateModel(self)

%         % Create Panda model
%               link(1) = Link([pi   0.333     0       -pi/2   0]);  
%               link(2) = Link([0     0        0        pi/2   0]);  
%               link(3) = Link([0   0.316      0        pi/2   0]);  
%               link(4) = Link([0     0      0.0825    -pi/2   0]); 
%               link(5) = Link([0   0.384   -0.0825    -pi/2   0]); 
%               link(6) = Link([0     0        0        pi/2   0]); 
% %               link(7) = Link([0     0      0.088      pi/2   0]); 
% 
%         % Incorporate joint limits
%               link(1).qlim = [-2.7437 2.7437];
%               link(2).qlim = [-1.7837 1.7837];
%               link(3).qlim = [-2.9007 2.9007];
%               link(4).qlim = [-3.0421 -0.1518];
%               link(5).qlim = [-2.8065 2.8065];
%               link(6).qlim = [0.5445 4.5169];
% %               link(7).qlim = [-3.0159 3.0159];

        %Franka Emika Panda bot DH paramerters
            link(1) = Link('d',0.333,'a',0,'alpha',0,'offset',0,'qlim', [-2.7437 2.7437]);
            link(2) = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim', [-1.7837,1.7837]);
            link(3) = Link('d',0.316,'a',0,'alpha',pi/2,'offset',0,'qlim', [-2.9007,2.9007]);
            link(4) = Link('d',0,'a',0.0825,'alpha',pi/2,'offset',0,'qlim', [-3.0421,-0.1518]);
            link(5) = Link('d',0.384,'a',-0.0825,'alpha',-pi/2,'offset',0,'qlim', [-2.8065,2.8065]);
            link(6) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim', [0.5445,4.5169]);
            link(7) = Link('d',0,'a',0.088,'alpha',pi/2,'offset',0,'qlim', [-3.0159,3.0159]);
            link(8) = Link('d',1.07,'a',0,'alpha',0,'offset',0,'qlim', [0,0]);
        
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end