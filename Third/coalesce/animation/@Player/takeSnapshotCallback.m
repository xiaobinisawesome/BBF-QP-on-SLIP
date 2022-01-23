function takeSnapshotCallback(this, varargin)
%TAKESNAPSHOTCALLBACK Take snapshot callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Save screen shot with default settings
    print(this.handles.fig, ...
        this.options.snapshot.format, ...
        this.options.snapshot.dpi, ...
        this.options.snapshot.fileName(this));
end % takeSnapshotCallback
