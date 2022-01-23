classdef Player < handle
%PLAYER Creates an animation player object.
%
% Description:
%   AnimationPlayer is a MATLAB object-oriented framework for creating and
%   playing back animations.
%
% Features:
%   * Playback features including, play, pause, stop, fast forward, rewind,
%     speed adjustment, frame seeking and frame by frame.
%   * Uses a object-oriented interface to make creating animations straight
%     forward and intuitive.
%   * Export capabilities for screen capture and movie export for a variety
%     of formats and resolutions.
%   * Save and load functionality for later viewing.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties
		handles % Defines main figure window and children handles
		options % Defines animation player options
		state % Defines the animation player state
	end % properties

	properties (SetAccess = 'private')
		scene % Defines the scene object to be played
		videoWriter % Defines the videoWriter object
		version = '1.0' % Object version number
	end % properties

	methods
		function this = Player(scene)
		%PLAYER Create player class constructor.
		%
		% Syntax:
		%   obj = Player(scene)
		%
		% Inputs:
		%   scene - Scene object

			% Initialize option and state variables
			this.state.currentTime = 0;
			this.state.endTime = 1;
			this.state.isOpen = false;
			this.state.isPlay = false;
			this.state.isExport = false;
			this.options.isLoop = false;
			this.options.isSmoothing = false;
			this.options.speed = 1;
			this.options.size = [0 0 1280 720];
			this.options.renderer = 'OpenGl';
			this.options.snapshot.format = '-dpng';
			this.options.snapshot.dpi = '-r300';
			this.options.snapshot.fileName = @(this) ['snapshot-' num2str(this.state.currentTime) '-of-' num2str(this.state.endTime)];
			this.options.video.format = 'MPEG-4';
			this.options.video.frameRate = 60;
			this.options.video.quality = 100;
			this.options.video.startTime = 0;
			this.options.video.endTime = this.state.endTime;

			% Create graphical user interface window
			this.initializeGui;

			% Check if a scene object has been specified
			if nargin == 1
				% Set animation player state variables
				this.scene = scene;
				this.state.isOpen = true;

				% Update graphical user interface
				this.updateGui;

				% Turn on smoothing
				this.smoothingCallback;

				% Turn on looping
				this.loopCallback;
                    
            end

			% User feedback
			fprintf('Player object constructed!\n');
        end
        
        
        %%% Xiaobin: try to time lapse 

        function delete(~)
		%DELETE Delete the object.
		%
		% Description:
		%   DELETE does not need to called directly, as it is called when
		%   the object is cleared.

			% User feedback
			fprintf('Player object deleted!\n');
		end % delete
	end % methods
end % classdef
