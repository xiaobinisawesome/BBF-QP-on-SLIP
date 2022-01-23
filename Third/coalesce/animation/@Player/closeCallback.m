function closeCallback(this, varargin)
%CLOSECALLBACK Close callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Check if any animations are open
    if this.state.isOpen
        % Stop animation
        this.stopCallback;

        % Close animation
        delete(this.handles.axes);

        % Update GUI
        set(this.handles.fig, 'KeyPressFcn', '');
        set(this.handles.seekBarSlider, 'Enable', 'off');
        set(this.handles.seekBarText, 'String', 'Seek Bar: ');
        set(this.handles.uimenu.saveAs, 'Enable', 'off');
        set(this.handles.uimenu.playback, 'Enable', 'off');
        set(this.handles.uimenu.video, 'Enable', 'off');

        % Set open/close state flag
        this.state.isOpen = false;
    end % if
end % closeCallback
