function keyPressCallback(this, varargin)
%KEYPRESSCALLBACK Keypress callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Get the current key pressed and enter state machine
    switch get(this.handles.fig, 'CurrentKey');
        % Increase playback speed
        case 'uparrow'
            this.fasterCallback;

        % Decrease playback speed
        case 'downarrow'
            this.slowerCallback;

        % Play/pause
        case 'space'
            this.playCallback;

        % Next frame
        case 'rightarrow'
            this.nextFrameCallback;

        % Previous frame
        case 'leftarrow'
            this.previousFrameCallback;

        otherwise
            % Do nothing
    end % switch
end % keyPressCallback
