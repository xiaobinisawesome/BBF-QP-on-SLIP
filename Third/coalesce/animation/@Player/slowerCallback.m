function slowerCallback (this, varargin)
%SLOWERCALLBACK Slower callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Reduce playback speed
    this.options.speed = this.options.speed/1.5;
end % slowerCallback
