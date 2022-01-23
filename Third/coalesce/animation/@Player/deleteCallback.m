function deleteCallback(this, varargin)
%DELETECALLBACK Delete callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Set state flags to stop playing and exporting
    this.state.isPlay = false;
    this.state.isExport = false;
end % deleteCallback
