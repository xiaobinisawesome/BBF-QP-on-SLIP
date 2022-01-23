function snapshotSettingsCallback(this, varargin)
%SNAPSHOTSETTINGSCALLBACK Snapshot settings callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Create dialog box
    prompt = {'Image Format (-djpeg, -dtiff, -dpng)' 'Image Resolution (-r<number>)'};
    name = 'Snapshot Settings';
    nLines = 1;
    defaultSettings = {this.options.snapshot.format this.options.snapshot.dpi};
    settings = inputdlg(prompt, name, nLines, defaultSettings);

    % Parse settings
    if ~isempty(settings)
        this.options.snapshot.format = settings{1};
        this.options.snapshot.dpi = settings{2};
    end % if
end % snapshotSettingsCallback
