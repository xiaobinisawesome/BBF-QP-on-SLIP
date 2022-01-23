function this = reset(this)
%RESET Reset graphics model object.
%
% Description:
%   Resets transformation matrix back to eye matrix.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Reset transformation matrix
	this.A =        [1	0	0	0;
			  		0	1	0	0;
			  		0	0	1	0;
			  		0	0	0	1];
end % reset
