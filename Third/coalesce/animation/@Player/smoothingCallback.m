function smoothingCallback(this, varargin)
%SMOOTHINGCALLBACK Smoothing callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Toggle smoothing state
    this.options.isSmoothing = ~this.options.isSmoothing;

    % Find the handles of all graphical object in axes
    children = findall(this.handles.axes);
    ind = ismember(get(children, 'Type'), {'surface' 'line' 'patch'});

    % Check current state
    if (this.options.isSmoothing)
        % Turn on
        set(this.handles.uimenu.smoothing, 'Checked', 'on');
        set(children(ind), 'LineSmoothing', 'on');
    else
        % Turn off
        set(this.handles.uimenu.smoothing, 'Checked', 'off');
        set(children(ind), 'LineSmoothing', 'off');
    end % if
end % smoothingCallback
