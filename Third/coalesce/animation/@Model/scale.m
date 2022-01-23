function this = scale(this, sx, sy, sz)
%SCALE Apply scaling operation on graphics model object.
%
% Description:
%   Performs an object scaling and updates the transformation matrix.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Transformation matrix
	S = [	sx	0		0		0;
		 		0		sy	0		0;
		 		0		0		sz	0;
		 		0		0		0		1];

	% Apply scaling to transformation matrix
	this.A = S*this.A;
end % scale
