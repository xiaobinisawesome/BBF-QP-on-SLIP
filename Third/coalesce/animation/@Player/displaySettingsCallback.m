function displaySettingsCallback(this, varargin)
%DISPLAYSETTINGSCALLBACK Display settings callback.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Create dialog box
    prompt = {'Renderer (OpenGl, Painters, ZBuffer', 'Frame Width', 'Frame Height'};
    name = 'Display Settings';
    nLines = 1;
    defaultSettings = {this.options.renderer num2str(this.options.size(3)) num2str(this.options.size(4))};
    settings = inputdlg(prompt, name, nLines, defaultSettings);

    % Parse settings
    if ~isempty(settings)
        this.options.renderer = settings{1};
        this.options.size(3:4) = [str2double(settings{2}) str2double(settings{2})];
    end % if

    set(this.handles.fig, 'Renderer', this.options.renderer);
    set(this.handles.fig, 'Position', this.options.size + [0 0 100 100]);
end % displaySettingsCallback
