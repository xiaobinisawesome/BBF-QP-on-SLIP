function playCallback(this, varargin)
%PLAYCALLBACK Play callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Toggle play button state
    this.state.isPlay = ~this.state.isPlay;

    % Check current state
    if (this.state.isPlay)
        % Play
        set(this.handles.uimenu.play, 'Checked', 'on');
        this.run;
    else
        % Pause
        set(this.handles.uimenu.play, 'Checked', 'off');
    end % if
    

end % playCallback
