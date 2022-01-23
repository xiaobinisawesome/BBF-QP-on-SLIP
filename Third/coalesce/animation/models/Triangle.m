classdef Triangle < Model
%TRIANGLE Creates a 2D triangle model object.
%
% Description:
%   Creates a 2D triangle object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		width@double scalar = 1
		height@double scalar = 1
		color = 'w'
	end % properties

	methods
		function this = Triangle(width, height, color)
		%TRIANGLE Creates a 2D triangle model object.
		%
		% Syntax:
		%   obj = Triangle
		%   obj = Triangle(width, height)
		%   obj = Triangle(width, height, color)
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
					this.color = color;
				otherwise
					error('Invalid number of input arguments.');
			end % switch

			% Parametric equations for a triangle
			this.x = [0 this.width this.width/2 0];
			this.y = [0 0 this.height 0];
			this.z = 0*this.x;

			% Create graphics object
			this.handle = patch(this.x, this.y, this.z, this.color,...
				'EdgeColor', 'r', ...
				'LineWidth', 2);
		end % Triangle
        
        function thisCopy = copy(this, varargin)
            thisCopy = Triangle;
            thisCopy.width = this.width;
            thisCopy.height = this.height;
            thisCopy.color = this.color;
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
            thisCopy.handle = patch(this.x, this.y, this.z, this.color,...
				'EdgeColor', 'k', ...
				'LineWidth', 2);
             if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
        end
	end % methods
end % classdef
