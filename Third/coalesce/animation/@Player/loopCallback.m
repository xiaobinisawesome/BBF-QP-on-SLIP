function loopCallback(this, varargin)
%LOOPCALLBACK Loop callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Toggle loop state
    this.options.isLoop = ~this.options.isLoop;

    % Check current state
    if (this.options.isLoop)
        % Turn on
        set(this.handles.uimenu.loop, 'Checked', 'on');
    else
        % Turn off
        set(this.handles.uimenu.loop, 'Checked', 'off');
    end % if
end % loopCallback
