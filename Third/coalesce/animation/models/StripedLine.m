classdef StripedLine < Model
%STRIPEDLINE Creates a diagonally striped line model object.
%
% Description:
%   Creates a striped line object using the Model superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		width@double scalar = 1
		spacing@double scalar = 0.025
	end % properties

	methods
		function this = StripedLine(width, spacing)
		%STRIPEDLINE Creates a striped line model object.
		%
		% Syntax:
		%   obj = StripedLine
		%   obj = StripedLine(width)
		%   obj = StripedLine(width, spacing)
		%
		% Copyright 2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Model;

			% Parse input arguments and set default properties
			switch nargin
			case 0
				% Do nothing
			case 1
				this.width= width;
			case 2
				this.width= width;
				this.spacing = spacing;
			otherwise
				error('Invalid number of input arguments.');
			end % switch

			% Equations for a striped line
			xGrid = 0:spacing:width; yGrid = 0*xGrid;
			this.x = reshape([xGrid; xGrid - spacing; nan*xGrid], 1, []);
			this.y = reshape([yGrid; yGrid - spacing; nan*yGrid], 1, []);
			this.x = [this.x 0 width];
			this.y = [this.y 0 0];
			this.z = 0*this.x;

			% Create graphics object
			this.handle = plot3(this.x, this.y, this.z, ...
				'Color', 'black', ...
				'LineStyle', '-', ...
				'LineWidth', 2);
		end % StripedLine
	end % methods
end % classdef
