classdef Scene < handle
%SCENE Graphical scene object class.
%
% Description:
%   This provides a superclass for defining graphical scenes using model
%   objects. The resulting object can be viewed using the Player class.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		startTime
		endTime
	end % properties

	methods (Abstract)
		this = initialize(this); % Initialize graphics objects
		this = update(this, t); % Update graphics objects
	end % methods

	methods
		function this = Scene(varargin)
		%SCENE Graphical scene object class.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Store total animation time
			switch nargin
			case 1
				this.startTime = min(varargin{1});
				this.endTime = max(varargin{1});
			case 2
				this.startTime = varargin{1};
				this.endTime = varargin{2};
			otherwise
				error('coalesce:animation:Scene', ...
					'Invalid number of input arguments.');
			end % switch
		end % Scene
	end % methods
end % classdef
