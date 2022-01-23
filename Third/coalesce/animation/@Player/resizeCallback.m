function resizeCallback(this, varargin)
% RESIZECALLBACK Resize callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Get current figure size
    screenSize = get(this.handles.fig, 'Position');

    % Resize figure objects
    set(this.handles.seekBarSlider, 'Position', [10 30 screenSize(3)-20 20]);
    set(this.handles.seekBarText, 'Position', [10 10 screenSize(3)-20 20]);
end % resizeCallback
