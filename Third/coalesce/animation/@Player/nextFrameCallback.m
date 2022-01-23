function nextFrameCallback (this, varargin)
%NEXTFRAMECALLBACK Next frame callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Advance to next frame
    this.state.currentTime = this.state.currentTime + this.options.speed/30;

    % Limit frame counter to be within first and last frame
    this.state.currentTime = max(min(this.state.currentTime, this.scene.endTime), this.scene.startTime);

    % Update seekbar slider and text to current frame
    set(this.handles.seekBarSlider, 'Value', this.state.currentTime);
    set(this.handles.seekBarText, 'String', ['Seek Bar: ' num2str(this.state.currentTime) ' / ' num2str(this.scene.endTime)]);

    % Draw scene at current frame
    this.scene.update(this.state.currentTime);
    
%     A = this.scene.mass.A;
%     x = A(1,4); y = A(2,4);
%     plot( x, y ,'o', 'Parent',  this.handles.addSub);
%     
end % nextFrameCallback
