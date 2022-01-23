classdef Cylinder < Model
%CYLINDER Creates a 3D cylinder model object.
%
% Description:
%   Creates a 3D cylinder object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		radius@double scalar = 0.1
		length@double scalar = 1
		color = 'w'
		nNodes@double scalar = 50
	end % properties

	methods
		function this = Cylinder(radius, length, color, nNodes)
		%CYLINDER Creates a 3D cylinder model object.
		%
		% Syntax:
		%   obj = Cylinder
		%   obj = Cylinder(radius)
		%   obj = Cylinder(radius, length)
		%   obj = Cylinder(radius, length, color)
		%   obj = Cylinder(radius, length, color, nNodes)
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

			% Parametric equations for a cylinder
			r = [0; 1; 1; 0]*this.radius;
			theta = (0:this.nNodes)/this.nNodes*2*pi;
			this.x = [0; 0; 1; 1]*this.length*ones(1,this.nNodes+1);
			this.y = r*sin(theta);
			this.z = r*cos(theta);

			% Create graphics object
			this.handle = surf(this.x, this.y, this.z, ...
				'EdgeColor', 'none', ...
				'FaceColor', this.color);
		end % Cylinder
        
        
        function thisCopy = copy(this, varargin)
            thisCopy = Cylinder;
            thisCopy.radius = this.radius;
            thisCopy.color = this.color;
            thisCopy.nNodes = this.nNodes;
            thisCopy.x = this.x;
            thisCopy.y = this.y;
            thisCopy.z = this.z;
            thisCopy.handle = surf(this.x, this.y, this.z, ...
				'EdgeColor', 'none', ...
				'FaceColor', this.color);
            
             if ~isempty(varargin)
                alpha = varargin{1};
                set(thisCopy.handle, 'EdgeAlpha', alpha)
                set(thisCopy.handle, 'FaceAlpha', alpha)
                
            end
        end
	end % methods
end % classdef
