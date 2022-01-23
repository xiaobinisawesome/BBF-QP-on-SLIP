classdef Model < handle
%MODEL Graphical model object class.
%
% Description:
%   This provides a superclass for defining graphical models with built-in
%   methods for transforming and animating objects.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		x@double % Original x coordinate data
		y@double % Original y coordinate data 
		z@double % Original z coordinate data
		A@double % Transformation matrix
		handle % Graphics handle
	end % properties

	methods
		function this = Model
		%MODEL Create graphics model object.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Set object properties
			this.A = [1	0	0	0;
					  		0	1	0	0;
					  		0	0	1	0;
					  		0	0	0	1];
		end % Model
	end % methods
end % classdef
