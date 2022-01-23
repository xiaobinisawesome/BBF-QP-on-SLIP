classdef RoundedSquare < Model
%ROUNDEDSQUARE Creates a 2D rounded square model object.
%
% Description:
%   Creates a 2D rounded square object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		width@double scalar = 1
		height@double scalar = 1
		radius@double scalar = 0.1
		color = 'w'
        edgeColor = 'k'
		nNodes@double scalar = 50
        lineStyle = '-'
	end % properties

	methods
		function this = RoundedSquare(width, height, radius, lineStyle, color, nNodes, FaceColor )
		%ROUNDEDSQAURE Creates a 2D rounded square model object.
		%
		% Syntax:
		%   obj = RoundedSquare
		%   obj = RoundedSquare(width, height)
		%   obj = RoundedSquare(width, height, radius)
		%   obj = RoundedSquare(width, height, radius, color)
		%   obj = RoundedSquare(width, height, radius, color, nNodes)
		%
		% Copyright 2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Model;

			% Parse input arguments and set default properties
			switch nargin
				case 0
					% Do nothing
				case 2
					this.width = width;
					this.height = height;
				case 3
					this.width = width;
					this.height = height;
					this.radius= radius;
				case 4
					this.width = width;
					this.height = height;
					this.radius = radius;
                    this.lineStyle = lineStyle;
				case 5
					this.width = width;
					this.height = height;
					this.radius = radius;
                    this.lineStyle = lineStyle;
					this.edgeColor = color;
                case 6
                    this.width = width;
                    this.height = height;
                    this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.edgeColor = color;
                    this.nNodes = nNodes;
                     case 7
                    this.width = width;
                    this.height = height;
                    this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.edgeColor = color;
                    this.nNodes = nNodes;
                    this.color = FaceColor;
				otherwise
					error('Invalid number of input arguments.');
			end % switch

			% Check radius size is appropriate
			this.radius = min([this.radius, this.width/2, this.height/2]);
			if this.radius <= 0
				error('Radius must be positive real number.');
			end % if

			% Parametric equations for a rounded square
			theta = (0:this.nNodes)/this.nNodes*2*pi;
			this.x = this.radius*sin(theta);
			this.y = this.radius*cos(theta);
			this.z = 0.*theta;
			this.x(this.x > 0) = this.x(this.x > 0) + this.width - 2*this.radius;
			this.y(this.y > 0) = this.y(this.y > 0) + this.height - 2*this.radius;
			this.x = this.x + this.radius;
			this.y = this.y + this.radius;
            
            this.setLength(this.height);
			% Create graphics object
			this.handle = patch(this.x, this.y, this.z, 'FaceColor', this.color,...
				'EdgeColor', this.edgeColor, ...
				'LineWidth', 2,...
                'LineStyle', this.lineStyle);
		end % RoundedSquare
        
        function this = setColor(this, faceColor, edgeColor) 
            this.handle = patch(this.x, this.y, this.z, faceColor,...
				'EdgeColor', edgeColor, ...
				'LineWidth', 2,...
                'LineStyle', this.lineStyle);
        end 
        
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
             set(thisCopy.handle, 'EdgeColor', this.edgeColor); 
            set(thisCopy.handle, 'LineStyle', this.lineStyle); 
             if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
        end
	end % methods
end % classdef
