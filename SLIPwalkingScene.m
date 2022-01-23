
classdef SLIPwalkingScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	% axes@double
  	axes
  	ground@StripedCurve
    
  	mass@Circle
    color = 'k'
    rod1@RoundedSquare
    rod2@RoundedSquare

  	foot1@Circle
    foot2@Circle

    spring1@Spring
    spring2@Spring
    
  	% response@Response
  	response
	end % properties

    properties % ABOUT THE APPEARANCE OF THE SPRING MASS
        massActuatorDistance = 0.05;
        springNormalLength = 0.2;
        
        rodwidth = 0.008; 
        footsize = 0.005;
    end
    
    properties  % objects positions. 
        zmx 
        zmy
        q1
        q2 
        zr1 
        zf1x 
        zf1y 
        zr2 
        zf2x 
        zf2y
        zs1
        zs2
        zterrain
        xterrain
        scaling = 3;
    end 
  % PUBLIC METHODS ========================================================
	methods
		function obj = SLIPwalkingScene(response, xterrain, zterrain, varargin)
			% Call superclass constructor
			obj = obj@Scene(response.t{1});

			% Store system response
			obj.response = response;
            obj.xterrain = xterrain; 
            obj.zterrain = zterrain;
              if ~isempty(varargin)
                  obj.color = varargin{1};
              end
		end % 

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle
			obj.axes = gca;

			% Define ground
            groundwidth = 60;
            obj.ground = StripedCurve(groundwidth, 0.5/80, obj.xterrain/obj.scaling, obj.zterrain/obj.scaling); % width spacing
            %obj.ground.translate(-groundwidth/2, -0.5/80 + 0.001, 0);
            obj.ground.translate(0, 0, 0);
			obj.ground.update;

			% Define mass
			obj.mass = Circle(0.015,'-',obj.color);
			obj.mass.translate(0.0, 0.0, 0);  %obj.mass.translate(-0.02/2, -0.02/2, 0);
			obj.mass.update;

			% Define rod
            
			obj.rod1 = RoundedSquare(obj.rodwidth, 0.05, 0.01/10, '-',obj.color);  %width, height, radius
			obj.rod1.translate(-obj.rodwidth/2, -0.0, 0); %obj.rod.translate(-0.01/2, -0.05/2, 0);
			obj.rod1.update;
             
            obj.rod2 = RoundedSquare(obj.rodwidth, 0.05, 0.01/10,'-',obj.color);  %width, height, radius
            obj.rod2.translate(-obj.rodwidth/2, -0.0, 0); %obj.rod.translate(-0.01/2, -0.05/2, 0);
            obj.rod2.update;
            
			% Define foot
			obj.foot1 = Circle(obj.footsize,'-',obj.color);
			obj.foot1.translate(-0.0, -0.00, 0);
			obj.foot1.update;
            
            obj.foot2 = Circle(obj.footsize,'-',obj.color);
			obj.foot2.translate(-0.0, -0.00, 0);
			obj.foot2.update;

            % Define spring 
            obj.spring1 = Spring(0.005,0.1,0.0005,obj.color);% radius, length, thickness
            obj.spring1.translate(-0.0, -0.00, 0);
            obj.spring1.rotate(0, 0, pi/2);
			obj.spring1.update;           

            obj.spring2 = Spring(0.005,0.1,0.0005,obj.color); % radius, length, thickness
            obj.spring2.translate(-0.0, -0.00, 0);
            obj.spring2.rotate(0, 0, pi/2);
			obj.spring2.update;
            
			% Axes properties
			view(0, 89.9); % Fixes issue where OpenGL will sometimes not draw objects
			axis off;
		end % initialize

        function obj = update(obj, t)
            %UPDATE Update graphical objects.
            obj = obj.updatePosition(t);
            obj = obj.updateObjects();    
        end
        
        function obj = updatePosition(obj,t)
			% Evaluate response at current time
			% [x, u] = obj.response.eval(t);
			response = obj.response;
            
            scaling = obj.scaling; 
            % mass position
            obj.zmx = interp1(response.t{1}, response.x/scaling, t);
            obj.zmy = interp1(response.t{1}, response.z/scaling, t) ;
            % leg angle
            obj.q1 = interp1(response.t{1}, response.q1, t); 
            obj.q2 = interp1(response.t{1}, response.q2, t); 
      
             % rod1 length
            obj.zr1 = interp1(response.t{1}, response.r1/scaling, t);
            
            % foot position
            obj.zf1x = interp1(response.t{1}, response.f1x/scaling, t); 
            obj.zf1y = interp1(response.t{1}, response.f1z/scaling, t); 
             
            % rod2 length
            obj.zr2 = interp1(response.t{1}, response.r2/scaling, t);
            
            % foot position
            obj.zf2x = interp1(response.t{1}, response.f2x/scaling, t); 
            obj.zf2y = interp1(response.t{1}, response.f2z/scaling, t); 
            
            % spring length
            obj.zs1 = obj.springNormalLength/scaling - interp1(response.t{1}, response.s1/scaling, t);
            obj.zs2 = obj.springNormalLength/scaling - interp1(response.t{1}, response.s2/scaling, t);
            
        end 
        
        function obj = updateObjects(obj)
            %% updates
			% Update mass
     		obj.mass.reset;
			obj.mass.translate(obj.zmx, obj.zmy + obj.mass.radius/2 +0.0075, 0);
			obj.mass.update;
%             
			% Update foot1
			obj.foot1.reset;
			obj.foot1.translate(obj.zf1x, obj.zf1y, 0);
			obj.foot1.update;

			% Update foot2
			obj.foot2.reset;
			obj.foot2.translate(obj.zf2x, obj.zf2y, 0);
			obj.foot2.update;

             % Update spring1
			obj.spring1.reset;
            obj.spring1.rotate(0, 0, pi/2-obj.q1);
            obj.spring1.translate(obj.zf1x, obj.zf1y, 0);
            obj.spring1.setLength(obj.zs1)
			obj.spring1.update;
            
            % Update rod1
			obj.rod1.reset;
            obj.rod1.rotate(0,0,-obj.q1);
			obj.rod1.translate(-obj.rodwidth/2 + obj.zf1x + sin(obj.q1)*obj.zs1, obj.zf1y + cos(obj.q1)*obj.zs1, 0); % y --> bottom of the rod ---> connected to spring top
            obj.rod1.setLength(obj.zr1 - obj.zs1)
			obj.rod1.update;
            
             % Update spring2
             obj.spring2.reset;
             obj.spring2.rotate(0, 0, pi/2-obj.q2);
             obj.spring2.translate(obj.zf2x, obj.zf2y, 0);
             obj.spring2.setLength(obj.zs2)
             obj.spring2.update;
             
              % Update rod1
			obj.rod2.reset;
            obj.rod2.rotate(0,0,-obj.q2);
			obj.rod2.translate(-obj.rodwidth/2 + obj.zf2x + sin(obj.q2)*obj.zs2, obj.zf2y + cos(obj.q2)*obj.zs2, 0); % y --> bottom of the rod ---> connected to spring top
            obj.rod2.setLength(obj.zr2 - obj.zs2)
            obj.rod2.update;
            % Set axes limits           
            ylim(obj.axes, [-0.1, 1]);
            ylim(obj.axes, [-0.8, 0.8]);

           % ylim(obj.axes, [-0.1, 0.8]);
            % ylim(obj.axes, [-0.3, 0.6]);
           % ylim(obj.axes, [-0.8, 0.4]);%%% -20 deg
            xlim(obj.axes, [-.2, max(obj.xterrain*1.2)/obj.scaling]);
            % xlim(obj.axes, [-.2, max(obj.xterrain*0.6)/obj.scaling]);

        end % update
    end
    
    methods 
%         function obj = setGround(obj, groundwidth)
%         			% Define ground
%             groundwidth = 20*groundwidth;
% 			obj.ground = StripedCurve(groundwidth, 0.5/80, obj.xterrain, obj.zterrain); % width spacing
% 			obj.ground.translate(-groundwidth/2, -0.5/80 + 0.001, 0);
% 			obj.ground.update;
%         end
%         
        function obj = updateAdd(obj, t, Xoffset, varargin)
            % use Xoffset to set distance between two 
             obj = obj.updatePosition(t);
            if ~isempty(varargin)
                alpha = varargin{1};
            else
                alpha = [];
            end
                    
            % add objects with new positions.
% 			% Update mass
            mass = obj.mass.copy(alpha); 
     		mass.reset;
			mass.translate(obj.zmx + Xoffset, obj.zmy + obj.mass.radius/2 +0.0075, 0);
			mass.update;
%             
			% Update foot1
            foot1 = obj.foot1.copy(alpha);
			foot1.reset;
			foot1.translate(obj.zf1x + Xoffset, obj.zf1y, 0);
			foot1.update;

			% Update foot2
            foot2 = obj.foot2.copy(alpha);
			foot2.reset;
			foot2.translate(obj.zf2x+ Xoffset, obj.zf2y, 0);
			foot2.update;

            % Update spring1
			spring1 = obj.spring1.copy(alpha/5); 
            spring1.reset;
            spring1.rotate(0, 0, pi/2-obj.q1);
            spring1.translate(obj.zf1x + Xoffset, obj.zf1y, 0);
            spring1.setLength(obj.zs1)
			spring1.update;
            
%             % Update rod1
            rod1 = obj.rod1.copy(alpha); 
			rod1.reset;
            rod1.rotate(0,0,-obj.q1);
			rod1.translate(-obj.rodwidth/2 + obj.zf1x + Xoffset + sin(obj.q1)*obj.zs1, obj.zf1y + cos(obj.q1)*obj.zs1, 0); % y --> bottom of the rod ---> connected to spring top
            rod1.setLength(obj.zr1 - obj.zs1)
			rod1.update;
%             
%             % Update spring2
            spring2 = obj.spring2.copy(alpha/5);
            spring2.reset;
            spring2.rotate(0, 0, pi/2-obj.q2);
            spring2.translate(obj.zf2x + Xoffset, obj.zf2y, 0);
            spring2.setLength(obj.zs2)
            spring2.update;
            
%             % Update rod1
            rod2 = obj.rod2.copy(alpha); 
			rod2.reset;
            rod2.rotate(0,0,-obj.q2);
			rod2.translate(-obj.rodwidth/2 + obj.zf2x+ Xoffset + sin(obj.q2)*obj.zs2, obj.zf2y + cos(obj.q2)*obj.zs2, 0); % y --> bottom of the rod ---> connected to spring top
            rod2.setLength(obj.zr2 - obj.zs2)
			rod2.update;
			% Set axes limits
%   			ylim(obj.axes, [-0.05, 0.30]);
% 			xlim(obj.axes, [-.05 3]);
        end
        
    end % methods
end % classdef