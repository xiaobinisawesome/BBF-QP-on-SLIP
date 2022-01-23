classdef steppingStone < Model

	properties
		width = 0.1
		height
        distance
		radius
		color = 'w'
        edgeColor = 'k'
		nNodes = 50
        lineStyle = '-'
	end % properties

	methods
		function this = steppingStone(distance, height, radius, lineStyle, color, nNodes)		%
		% Syntax:
		%   obj = steppingStone
		%   obj = steppingStone(width, height)
		%   obj = steppingStone(width, height, radius)
		%   obj = steppingStone(width, height, radius, color)
		%   obj = steppingStone(width, height, radius, color, nNodes)
		
        %%% width is a vec of the width of the stones
        %%% height is a vec of the height of the stones
        %%% radius is a constant 
			this = this@Model;

			% Parse input arguments and set default properties
			switch nargin
				case 0
					% Do nothing
				case 2
					this.distance = distance;
					this.height = height;
				case 3
					this.distance = distance;
					this.height = height;
					this.radius= radius;
				case 4
					this.distance = distance;
					this.height = height;
					this.radius = radius;
                    this.lineStyle = lineStyle;
				case 5
					this.distance = distance;
					this.height = height;
					this.radius = radius;
                    this.lineStyle = lineStyle;
					this.color = color;
                case 6
                    this.distance = distance;
                    this.height = height;
                    this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.color = color;
                    this.nNodes = nNodes;
				otherwise
					error('Invalid number of input arguments.');
			end % switch
            
            this.width = this.radius*2.5; 
			% Check radius size is appropriate
            for i = 1:length(this.width)
               
                % Parametric equations for a rounded square
                theta = (0:this.nNodes)/this.nNodes*2*pi;
                this.x = this.radius*sin(theta);
                this.y = this.radius*cos(theta);
                this.z = 0.*theta;
                this.x(this.x > 0) = this.x(this.x > 0) + this.width - 2*this.radius;
                this.y(this.y > 0) = this.y(this.y > 0) + this.height(i) - 2*this.radius;
                this.x = this.x + this.radius;
                this.y = this.y + this.radius;
                
                this.setLength(this.height(i));
                % Create graphics object
                this.handle = patch(this.x, this.y, this.z, 'w',...
                    'EdgeColor', this.color, ...
                    'LineWidth', 2,...
                    'LineStyle', this.lineStyle);
            end
		end % RoundedSquare
        
        
        function setLength(this, length)
            			% Parametric equations for a rounded square\
            this.height = length; 
			theta = (0:this.nNodes)/this.nNodes*2*pi;
			this.x = this.radius*sin(theta);
			this.y = this.radius*cos(theta);
			this.z = 0.*theta;
			this.x(this.x > 0) = this.x(this.x > 0) + this.width - 2*this.radius;
			this.y(this.y > 0) = this.y(this.y > 0) + this.height - 2*this.radius;
			this.x = this.x + this.radius;
			this.y = this.y + this.radius;

        end
        
        function thisCopy = copy(this, varargin)
            thisCopy = RoundedSquare;
            
            thisCopy.width = this.width;
            thisCopy.height = this.height;
            thisCopy.radius = this.radius;
            thisCopy.lineStyle = this.lineStyle;
            thisCopy.color = this.color;
            thisCopy.nNodes = this.nNodes;
            
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
                     set(thisCopy.handle, 'EdgeColor', this.color); 
            set(thisCopy.handle, 'LineStyle', this.lineStyle); 
             if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
        end
	end % methods
end % classdef
