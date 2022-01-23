function update(this)
%UPDATE Update graphics model object.
%
% Description:
%   Applies the current transformation matrix to the object coordinates and
%   updates the handle graphics object.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Apply transformation matrix
	[xData, yData, zData] = this.transform;

	% Update graphics object
    % handleCopy = copy(this.handle); 
    
	set(this.handle, ...
		'XData', xData, ...
		'YData', yData, ...
		'ZData', zData);
end % update
