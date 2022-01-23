classdef Sphere < Model
%SPHERE Creates a 3D sphere model object.
%
% Description:
%   Creates a 3D sphere object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		radius@double scalar = 0.1
		color = 'w'
		nNodes@double scalar = 50
	end % properties

	methods
		function this = Sphere(radius, color, nNodes)
		%SPHERE Creates a 3D sphere model object.
		%
		% Syntax:
		%   obj = Sphere
		%   obj = Sphere(radius)
		%   obj = Sphere(radius, color)
		%   obj = Sphere(radius, color, n)
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
					this.color = color;
				case 3
					this.radius = radius;
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

			% Create graphics object
			this.handle = surf(this.x, this.y, this.z, ...
				'EdgeColor', 'none', ...
				'FaceColor', this.color);
        end % Sphere
        
        function thisCopy = copy(this)
            thisCopy = Sphere;
            thisCopy.radius = this.radius;
            thisCopy.color = this.color;
            thisCopy.nNodes = this.nNodes;
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
            thisCopy.handle =  surf(this.x, this.y, this.z, ...
                'EdgeColor', 'none', ...
                'FaceColor', this.color);
        end
	end % methods
end % classdef
