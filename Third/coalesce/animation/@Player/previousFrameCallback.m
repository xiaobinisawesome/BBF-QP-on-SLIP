function previousFrameCallback (this, varargin)
%PREVIOUSFRAMECALLBACK Previous frame callback.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Regress to previous frame
    this.state.currentTime = this.state.currentTime - this.options.speed/30;

    % Limit frame counter to be within first and last frame
    this.state.currentTime = max(min(this.state.currentTime, this.scene.endTime), this.scene.startTime);

    % Update seekbar slider and text to current frame
    set(this.handles.seekBarSlider, 'Value', this.state.currentTime);
    set(this.handles.seekBarText, 'String', ['Seek Bar: ' num2str(this.state.currentTime) ' / ' num2str(this.scene.endTime)]);

    % Draw scene at current frame
    this.scene.update(this.state.currentTime);
end % previousFrameCallback
