function openCallback(this, varargin)
%OPENCALLBACK Open callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Check if anything is already open
    if this.state.isOpen
        % Prompt user for feedback
        choice = questdlg('Close scene?', 'Player', 'Yes', 'No', 'No');

        % If no then leave function otherwise continue loading
        if strcmp(choice, 'No')
            return;
        end % if
    end % if

    % Standard open file dialog box
    [fileName, pathName] = uigetfile('*.mat');

    % Only continue if a file was selected
    if ~(fileName == 0)
        % Close any open animations
        this.closeCallback;

        % Load scene data from a .mat file
        temp = load([pathName fileName], 'scene');
        this.scene = temp.scene;

        % Update graphical user interface and display scene
        this.updateGUI;

        % Set open/close state flag
        this.state.isOpen = true;
    end % if
end % openCallback
