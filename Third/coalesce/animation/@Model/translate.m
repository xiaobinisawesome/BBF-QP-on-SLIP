function this = translate(this, tx, ty, tz)
%TRANSLATE Apply translation operation on graphics model object.
%
% Description:
%   Performs an object translation and updates the transformation matrix.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Transformation matrix
	T = [	1		0		0		tx;
		 		0		1		0		ty;
		 		0		0		1		tz;
		 		0		0		0		1];

   % Apply translation to transformation matrix
   this.A = T*this.A;
end % translate
