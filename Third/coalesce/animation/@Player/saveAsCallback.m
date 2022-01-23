function saveAsCallback(this, varargin)
%SAVEASCALLBACK Save as callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Standard open file dialog box
    [fileName, pathName] = uiputfile('*.mat');

    % Only continue if a file was selected
    if ~(fileName == 0)
        % Save scene data into a .mat file
        scene = this.scene;
        save([pathName fileName], 'scene');
    end % if
end % saveAsCallback
