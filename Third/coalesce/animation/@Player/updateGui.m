function updateGui(this)
%UPDATEGUI Update graphical user
%
% Copyright 2013-2014 Mikhail S. Jones

    % Set state variables and options
    this.state.totalTime = this.scene.endTime;
    this.options.speed = 1;
    this.options.video.startTime = this.scene.startTime;
    this.options.video.endTime = this.state.totalTime;

    % Reset axes
%         
%     addSub = axes('Parent', this.handles.fig);
%     subplot(2,1,1)
%     plot(0:0.1:1,0:0.1:1,'o', 'Parent', addSub);
%     this.handles = figure;
%     this.handles = copyobj(this.handles, addSub);

%%%%%%%%%%%%%%%%%% clearify addition subplots
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Xiaobin: not sure if it is best way
%     addSub = subplot(2,1,1, 'Parent', this.handles.fig);
%     this.handles.addSub = addSub;

%     %%%%%%%%%%%%%%%%
%     set(this.handles.addSub, 'DataAspectRatio', [1, 1, 1])


    this.handles.axes = axes(...
        'Units', 'pixels', ...
        'OuterPosition', this.options.size + [50 50 2 2], ...
        'Position', this.options.size + [50 50 2 2], ...
        'DataAspectRatio', [1, 1, 1], ...
        'NextPlot', 'add', ...
        'Parent', this.handles.fig);

    % Reset user interface
    set(this.handles.fig, 'KeyPressFcn', @this.keyPressCallback);
    set(this.handles.seekBarSlider, ...
        'Min', this.scene.startTime, ...
        'Max', this.scene.endTime, ...
        'SliderStep', [1/30 1], ...
        'Enable', 'on');
    set(this.handles.uimenu.saveAs,   'Enable', 'on');
    set(this.handles.uimenu.playback, 'Enable', 'on');
    set(this.handles.uimenu.video,    'Enable', 'on');

    % Initialize scene
    this.scene.initialize;
    this.scene.update(this.scene.startTime);
    

end % updateGui
