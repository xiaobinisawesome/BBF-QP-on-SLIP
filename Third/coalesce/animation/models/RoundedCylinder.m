classdef RoundedCylinder < Model
%ROUNDEDCYLINDER Creates a 3D rounded cylinder model object.
%
% Description:
%   Creates a 3D rounded cylinder object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		radius@double scalar = 0.1
		length@double scalar = 1
		color = 'w'
		nNodes@double scalar = 50
	end % properties

	methods
		function this = RoundedCylinder(length, radius, color, nNodes)
		%ROUNDEDCYLINDER Creates a 3D rounded cylinder model object.
		%
		% Syntax:
		%   obj = RoundedCylinder
		%   obj = RoundedCylinder(radius)
		%   obj = RoundedCylinder(radius, length)
		%   obj = RoundedCylinder(radius, length, color)
		%   obj = RoundedCylinder(radius, length, color, nNodes)
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
					this.length = length;
				case 3
					this.radius = radius;
					this.length = length;
					this.color = color;
				case 4
					this.radius = radius;
					this.length = length;
					this.color = color;
					this.nNodes = nNodes;
				otherwise
					error('Invalid number of input arguments.');
			end % switch

			% Parametric equations for a sphere
			theta = (0:this.nNodes)/this.nNodes*2*pi;
			phi = (0:this.nNodes)'/this.nNodes*pi;
			this.x = this.radius*cos(phi)*ones(1,this.nNodes+1);
			this.y = this.radius*sin(phi)*sin(theta);
			this.z = this.radius*sin(phi)*cos(theta);
			this.x(this.x > 0) = this.length - 2*this.radius + this.x(this.x > 0);
			this.x = this.x + this.radius;

			% Create graphics object
			this.handle = surf(this.x, this.y, this.z, ...
				'EdgeColor', 'k', ...
				'FaceColor', this.color);
		end % RoundedCylinder
        
        
        function thisCopy = copy(this, varargin)
            thisCopy = RoundedCylinder;
            thisCopy.radius = this.radius;
            thisCopy.color = this.color;
            thisCopy.nNodes = this.nNodes;
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
            thisCopy.handle = surf(this.x, this.y, this.z, ...
				'EdgeColor', 'k', ...
				'FaceColor', this.color);
             if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
        end
        
	end % methods
end % classdef
