clear all; close all; clc;
addpath('functions/')

%% Load data
load('data/beziers/allBeziersHuman.mat')
ab = allBeziers;

load('data/human/timeEvents.mat')
load('data/human/xcomEvents.mat')

%% Some settings for later down the line
colors = {'r',  'b',   'g',  'c',   'k',  'b',   'g',  'c',   'k'};
downsteps = [0 -0.025 -0.05 -0.075 -0.10 -0.025 -0.05 -0.075 -0.10];
exps = fieldnames(ab);
phs = fieldnames(ab.exp00);
Tphs = {'Ts1','Ts2','Ts3'};


%% Some initial plotting
figure
teval = linspace(0,1,100);
for ph = 1:length(phs)
    subplot(2,6,(ph-1)*2+1); hold on; grid on;
    for exp = [1 2 3 4 5]
        timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax_SSP;
        time = linspace(0,timeMax,100);
        plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_SSP,teval))
    end
    xlabel('time')
    title('expected')
    
    subplot(2,6,(ph-1)*2+2); hold on; grid on;
    try
        for exp = [1 2 3 4 5]
            timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax - ...
                timeEvents.(exps{exp}).(phs{ph}).timeMax_SSP;
            time = linspace(0,timeMax,100);
            plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_DSP_st,teval))
            plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_DSP_sw,teval))
        end
        xlabel('time')
        title('expected')
    end
    legend('exp00','exp25','exp50','exp75','exp100')
    
    
    
    subplot(2,6,(ph-1)*2+1+6); hold on; grid on;
    for exp = [1 6 7 8 9]
        timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax_SSP;
        time = linspace(0,timeMax,100);
        plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_SSP,teval))
    end
    xlabel('time')
    title('expected')
    
    subplot(2,6,(ph-1)*2+2+6); hold on; grid on;
    try
        for exp = [1 6 7 8 9]
            timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax - ...
                timeEvents.(exps{exp}).(phs{ph}).timeMax_SSP;
            time = linspace(0,timeMax,100);
            plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_DSP_st,teval))
            plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_grf_DSP_sw,teval))
        end
        xlabel('time')
        title('expected')
    end
    
    legend('exp00','unexp25','unexp50','unexp75','unexp100')
end



%% Steptime scaling
load('data/human/bezierStepTimes.mat')

%% Bezier scaling
bvFits = struct;
bvFits.exp = struct;
bvFits.unexp = struct;

for ph = 1:length(phs)
    %%% expected
    % SSP
    for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_SSP) % num of Bezier parameters
        bv_param = [];
        for exp = [1 2 3 4 5] % [1 2 3 4 5]
            bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_SSP(ii)];
        end
        bvFits.exp.(phs{ph}).grf_SSP{ii} = polyfit(downsteps(1:5),bv_param,1);
    end
    % DSP st
    try
        for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_DSP_st) % num of Bezier parameters
            bv_param = [];
            for exp = [1 2 3 4 5] % [1 2 3 4 5]
                bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_DSP_st(ii)];
            end
            bvFits.exp.(phs{ph}).grf_DSP_st{ii} = polyfit(downsteps(1:5),bv_param,1);
        end
        % DSP sw
        for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_DSP_sw) % num of Bezier parameters
            bv_param = [];
            for exp = [1 2 3 4 5] % [1 2 3 4 5]
                bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_DSP_sw(ii)];
            end
            bvFits.exp.(phs{ph}).grf_DSP_sw{ii} = polyfit(downsteps(1:5),bv_param,1);
        end
    end
    
    %%% unexpected
    % SSP
    for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_SSP) % num of Bezier parameters
        bv_param = [];
        for exp = [1 6 7 8 9] % [1 2 3 4 5]
            bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_SSP(ii)];
        end
        bvFits.unexp.(phs{ph}).grf_SSP{ii} = polyfit(downsteps(1:5),bv_param,1);
    end
    % DSP st
    try
        for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_DSP_st) % num of Bezier parameters
            bv_param = [];
            for exp = [1 6 7 8 9] % [1 2 3 4 5]
                bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_DSP_st(ii)];
            end
            bvFits.unexp.(phs{ph}).grf_DSP_st{ii} = polyfit(downsteps(1:5),bv_param,1);
        end
        % DSP sw
        for ii = 1:length(ab.exp00.(phs{ph}).bv_grf_DSP_sw) % num of Bezier parameters
            bv_param = [];
            for exp = [1 6 7 8 9] % [1 2 3 4 5]
                bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_grf_DSP_sw(ii)];
            end
            bvFits.unexp.(phs{ph}).grf_DSP_sw{ii} = polyfit(downsteps(1:5),bv_param,1);
        end
    end
end
linkaxes

%% Plot Fgrf for downsteps
hs = linspace(0,-0.30,30);

figure
for ph = 1:length(phs)
    %%% expected
    % SSP
    subplot(2,6,(ph-1)*2+1); hold on; grid on;
    n = length(ab.exp00.(phs{ph}).bv_grf_SSP);
    bv = zeros(1,n);
    for h = hs
        for ii = 1:n
            bv(ii) = polyval(bvFits.exp.(phs{ph}).grf_SSP{ii},h);
        end
        time = linspace(0,polyval(bezierStepTimes.exp.(Tphs{ph}),h),100);
        plot(time,bezier2(bv,teval),'r');
    end
    title('expected SSP')
    % DSP st
    try
        subplot(2,6,(ph-1)*2+2); hold on; grid on;
        n = length(ab.exp00.(phs{ph}).bv_grf_DSP_st);
        bv_st = zeros(1,n);
        bv_sw = zeros(1,n);
        for h = hs
            for ii = 1:n
                bv_st(ii) = polyval(bvFits.exp.(phs{ph}).grf_DSP_st{ii},h);
                bv_sw(ii) = polyval(bvFits.exp.(phs{ph}).grf_DSP_sw{ii},h);
            end
            time = linspace(0,polyval(bezierStepTimes.exp.(Tphs{ph}),h),100);
            plot(time,bezier2(bv_st,teval),'r');
            plot(time,bezier2(bv_sw,teval),'b');
        end
        title('expected DSP')
    end
    
    %%% unexpected
    % SSP
    subplot(2,6,(ph-1)*2+1+6); hold on; grid on;
    n = length(ab.exp00.(phs{ph}).bv_grf_SSP);
    bv = zeros(1,n);
    for h = hs
        for ii = 1:n
            bv(ii) = polyval(bvFits.unexp.(phs{ph}).grf_SSP{ii},h);
        end
        time = linspace(0,polyval(bezierStepTimes.unexp.(Tphs{ph}),h),100);
        plot(time,bezier2(bv,teval),'r');
    end
    title('unexpected SSP')
    % DSP st
    try
        subplot(2,6,(ph-1)*2+2+6); hold on; grid on;
        n = length(ab.exp00.(phs{ph}).bv_grf_DSP_st);
        bv_st = zeros(1,n);
        bv_sw = zeros(1,n);
        for h = hs
            for ii = 1:n
                bv_st(ii) = polyval(bvFits.unexp.(phs{ph}).grf_DSP_st{ii},h);
                bv_sw(ii) = polyval(bvFits.unexp.(phs{ph}).grf_DSP_sw{ii},h);
            end
            time = linspace(0,polyval(bezierStepTimes.unexp.(Tphs{ph}),h),100);
            plot(time,bezier2(bv_st,teval),'r');
            plot(time,bezier2(bv_sw,teval),'b');
        end
        title('unexpected DSP')
    end
end
linkaxes

bezierInterpolation = bvFits;
save('data/outputs/bezierInterpolation.mat','bezierInterpolation')



