classdef Circle < Model
%CIRCLE Creates a 2D circle model object.
%
% Description:
%   Creates a 2D circle object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		radius@double scalar = 0.1
		color = 'w'
        facecolor = 'k'
		nNodes@double scalar = 50
        lineStyle = '-'
	end % properties

	methods
		function this = Circle(radius, lineStyle, color, facecolor, nNodes)
		%CIRCLE Creates a 2D circle model object.
		%
		% Syntax:
		%   obj = Circle
		%   obj = Circle(radius)
		%   obj = Circle(radius, color)
		%   obj = Circle(radius, color, n)
		%
		% Copyright 2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Model;

			% Parse input arguments and set default properties
			switch nargin
				case 0
					% Do nothing
				case 1
					this.radius = radius;
				case 2
					this.radius = radius;
                    this.lineStyle = lineStyle;
                case 3
                    this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.color = color;
                case 4
                    this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.color = color;
                    this.facecolor = facecolor;
				case 5
					this.radius = radius;
                    this.lineStyle = lineStyle;
                    this.color = color;
                    this.facecolor = facecolor;
					this.nNodes = nNodes;
				otherwise
					error('Invalid number of input arguments.');
			end % switch

			% Parametric equations for a circle
			theta = (0:this.nNodes)/this.nNodes*2*pi;
			this.x = this.radius*sin(theta);
			this.y = this.radius*cos(theta);
			this.z = 0*theta;

			% Create graphics object
			this.handle = patch(this.x, this.y, this.z, this.facecolor, ...
				'EdgeColor', this.color, ...
				'LineWidth', 2, ...
                'LineStyle', this.lineStyle);
		end % Circle
        
        function thisCopy = copy(this, varargin) 
            thisCopy = Circle;
            thisCopy.radius = this.radius;
            thisCopy.lineStyle = this.lineStyle;
            thisCopy.color = this.color;
            thisCopy.nNodes = this.nNodes;
            thisCopy.x = this.x;
			thisCopy.y = this.y;
			thisCopy.z = this.z;
            set(thisCopy.handle, 'EdgeColor', this.color); 
            set(thisCopy.handle, 'FaceColor', this.facecolor); 
            set(thisCopy.handle, 'LineStyle', this.lineStyle); 
            if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
%             thisCopy.handle = patch(this.x, this.y, this.z, this.color, ...
%                 'EdgeColor', 'k', ...
%                 'LineWidth', 2, ...
%                 'LineStyle', this.lineStyle);
        end
	end % methods
end % classdef
