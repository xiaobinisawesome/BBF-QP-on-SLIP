function [x, y, z] = transform(this)
%TRANSFORM Apply transformation to graphics model object.
%
% Description:
%   Applies the current transformation matrix to the object coordinates.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Loop through vertexes
	for r = size(this.x, 1):-1:1
		for c = size(this.x, 2):-1:1
			% Pull single vertex
			v = [this.x(r,c); this.y(r,c); this.z(r,c); 1];

			% Apply transformation
			vt = this.A*v;

			% Store the resulting vertex
			x(r,c) = vt(1);
			y(r,c) = vt(2);
			z(r,c) = vt(3);
		end % for
	end % for
end % transform
