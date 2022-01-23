function stopCallback(this, varargin)
%STOPCALLBACK Stop callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % If the animation is playing then toggle play button to pause
    if (this.state.isPlay)
        this.playCallback;
    end % if

    % Rewind animation
    this.rewindCallback;
end % stopCallback
