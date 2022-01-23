function videoSettingsCallback(this, varargin)
%VIDEOSETTINGSCALLBACK Video settings callback.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Create dialog box
	prompt = {'Video Format (Archival, Motion JPEG AVI, Motion JPEG 2000, MPEG-4, Uncompressed AVI)' 'Frame Rate (fps)' 'Video Quality (0 - 100)' 'Start Time' 'End Time'};
	name = 'Video Export Settings';
	nLines = 1;
	defaultSettings = {this.options.video.format num2str(this.options.video.frameRate) num2str(this.options.video.quality) num2str(this.options.video.startTime) num2str(this.options.video.endTime)};
	settings = inputdlg(prompt, name, nLines, defaultSettings);

	% Parse settings
	if ~isempty(settings)
		this.options.video.format = settings{1};
		this.options.video.frameRate = str2double(settings{2});
		this.options.video.quality = str2double(settings{3});
		this.options.video.startTime = str2double(settings{4});
		this.options.video.endTime = str2double(settings{5});
	end % if
end % videoSettingsCallback
