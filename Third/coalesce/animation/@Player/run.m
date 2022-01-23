function run(this)
%RUN Run animation.
%
% Copyright 2013-2014 Mikhail S. Jones

    % Initialize clock timer
    dt = tic;

    % Animation loop
    while this.state.isPlay
        % Update frame counter
        this.state.currentTime = get(this.handles.seekBarSlider, 'Value') + toc(dt)*this.options.speed;

        % Update clock timer
        dt = tic;

        if (this.options.isLoop)
            % Loop frame counter
            this.state.currentTime = this.scene.startTime + mod(this.state.currentTime - this.scene.startTime, this.scene.endTime - this.scene.startTime);
        else
            % Limit frame counter to be within first and last frame
            this.state.currentTime = max(min(this.state.currentTime, this.scene.endTime), this.scene.startTime);

            % If finished with animation then toggle pause
            if (this.state.currentTime == this.scene.startTime) || (this.state.currentTime == this.scene.endTime)
                this.playCallback;
            end % if
        end % if

        % Update seek bar slider and text to current frame
        set(this.handles.seekBarSlider, 'Value', this.state.currentTime);
        set(this.handles.seekBarText, 'String', ['Seek Bar: ' num2str(this.state.currentTime) ' / ' num2str(this.scene.endTime)]);

        % Draw scene at current frame
        this.scene.update(this.state.currentTime);
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%  Xiaobin's custom %%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%% To add additional plots
         %this.scene.addPlot(this);
        % Flush pending graphics events
        drawnow;
    end % while
end % run
