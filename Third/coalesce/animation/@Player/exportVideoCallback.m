function exportVideoCallback(this, varargin)
%EXPORTVIDEOCALLBACK Export video callback function.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Prompt user for path and file name
    [fileName, pathName] = uiputfile;

    % Only continue if a file was selected
    if ~(fileName == 0)
        % Stop animation
        this.stopCallback;

        % Initialize at start frame
        this.state.currentTime = this.options.video.startTime;
        set(this.handles.seekBarSlider, 'Value', this.state.currentTime);
        set(this.handles.seekBarText, 'String', ['Seek Bar: ' num2str(this.state.currentTime) ' / ' num2str(this.scene.endTime)]);
        this.scene.update(this.state.currentTime);

        % Set video export flag
        this.state.isExport = true;

        % Find and disable all ui objects in figure
        children = findall(this.handles.fig);
        ind = ismember(get(children, 'Type'), {'uimenu' 'uicontrol'});
        set(children(ind), 'Enable', 'off');
        set(this.handles.fig, 'KeyPressFcn', '');

        % Construct video writer object
%         this.videoWriter = VideoWriter([pathName, fileName], this.options.video.format);
        this.videoWriter = VideoWriter([pathName, fileName], 'Motion JPEG AVI');
        this.videoWriter.FrameRate = this.options.video.frameRate;
        try
            this.videoWriter.Quality = this.options.video.quality;
        catch
            % Do nothing
        end % try
        open(this.videoWriter);

        % Animation loop
        while this.state.isExport
            % Capture frame image for playback
           % frame = getframe(this.handles.axes, this.options.size + [1 1 0 0]);
             size = this.handles.axes.Position;
             frame = getframe(gcf); %getframe(this.handles.axes, [51, 51, 1200, 700] );
           %  frame.cdata = frame.cdata(1:600, 1:1200,:); 
             writeVideo(this.videoWriter, frame);

            % Update frame counter
            this.state.currentTime = this.state.currentTime + 1/this.options.video.frameRate;

            % Limit frame counter to be within first and last frame
            this.state.currentTime = max(min(this.state.currentTime, this.options.video.endTime), this.options.video.startTime);

            % If finished with animation then toggle export flag
            if (this.state.currentTime == this.options.video.startTime) || (this.state.currentTime == this.options.video.endTime)
                this.state.isExport = false;
            else

                % Update seekbar slider and text to current frame
                set(this.handles.seekBarSlider, 'Value', this.state.currentTime);
                set(this.handles.seekBarText, 'String', ['Seek Bar: ' num2str(this.state.currentTime) ' / ' num2str(this.scene.endTime)]);

                % Draw scene at current frame
                this.scene.update(this.state.currentTime);

                % Flush pending graphics events
                drawnow;
            end % if
        end % while

        % Close video writer object
        close(this.videoWriter);

        % Check if figure still exists
        if ishandle(this.handles.fig)
            % Re-enable all ui objects in figure
            set(children(ind), 'Enable', 'on');

            % Re-enable figure keypress functions
            set(this.handles.fig, 'KeyPressFcn', @this.keyPressCallback);
        end % if

        % User feedback
        msgbox('Done exporting video!', 'Player');
    end % if
end % exportVideoCallback
