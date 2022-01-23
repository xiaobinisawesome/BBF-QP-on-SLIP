function this = rotate(this, rx, ry, rz)
%ROTATE Apply rotation operation on graphics model object.
%
% Description:
%   Performs an object rotation and updates the transformation matrix.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Rotation about x-axis transformation matrix
	cx = cos(rx); sx = sin(rx);
	Rx = [1		0		0		0;
		  	0		cx	-sx	0;
		  	0		sx	cx	0;
		  	0		0		0		1];

	% Rotation about y-axis transformation matrix
	cy = cos(ry); sy = sin(ry);
	Ry = [cy	0		sy	0;
		  	0		1		0		0;
		  	-sy	0		cy	0;
		  	0		0		0		1];

	% Rotation about z-axis transformation matrix
	cz = cos(rz); sz = sin(rz);
	Rz = [cz	-sz	0		0;
		  	sz	cz	0		0;
		  	0		0		1		0;
		  	0		0		0		1];

	% Final rotation transformation
	R = Rz*Ry*Rx;

	% Apply rotation to transformation matrix
   this.A = R*this.A;
end % rotate
