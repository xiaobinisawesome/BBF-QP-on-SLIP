clear all; close all; clc;
addpath('functions/')

%% Load data
load('data/beziersRaw/allBeziersHuman.mat')
ab = allBeziers;

load('data/human/timeEvents.mat')
load('data/human/xcomEvents.mat')

%% Some settings for later down the line
colors = {'r',  'b',   'g',  'c',   'k',  'b',   'g',  'c',   'k'};
downsteps = [0 -0.025 -0.05 -0.075 -0.10 -0.025 -0.05 -0.075 -0.10];
exps = fieldnames(ab);
phs = fieldnames(ab.exp00);

%% Set first two indices of zcom equal
% db.exp00.phase1.bv_zcom(2) = db.exp00.phase1.bv_zcom(1);
% for exp = 1:length(exps)
%     db.(exps{exp}).('phase1').bv_zcom(1:2) = db.exp00.('phase1').bv_zcom(1:2);
%     db.(exps{exp}).('phase3').bv_zcom(4:5) = db.exp00.('phase3').bv_zcom(4:5);
% end


%% Some initial plotting
figure
teval = linspace(0,1,100);
for ph = 1:length(phs)
    subplot(2,6,ph); hold on; grid on;
    for exp = [1 2 3 4 5]
%         timeMax = db.(exps{exp}).(phs{ph}).timeMax;
        timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax;
        time = linspace(0,timeMax,100);
        plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
    end
    xlabel('time')
    title('expected')
    
    subplot(2,6,ph+6); hold on; grid on;
    for exp = [1 2 3 4 5]
        xcom = bezier2(ab.(exps{exp}).(phs{ph}).bv_xcom,teval);
        plot(xcom,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
    end
    xlabel('xcom')
    legend('exp00','exp25','exp50','exp75','exp100')
    
    subplot(2,6,ph+3); hold on; grid on;
    for exp = [1 6 7 8 9]
%         timeMax = db.(exps{exp}).(phs{ph}).timeMax;
        timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax;
        time = linspace(0,timeMax,100);
        plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
    end
    xlabel('time')
    title('unexpected')
    
    subplot(2,6,ph+9); hold on; grid on;
    for exp = [1 6 7 8 9]
        xcom = bezier2(ab.(exps{exp}).(phs{ph}).bv_xcom,teval);
        plot(xcom,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
    end
    xlabel('xcom')
    legend('unexp00','unexp25','unexp50','unexp75','unexp100')
end



%% Steptime scaling
stepTimes = struct;
stepTimes.exp.phase1 = [];
stepTimes.exp.phase2 = [];
stepTimes.exp.phase3 = [];
stepTimes.unexp.phase1 = [];
stepTimes.unexp.phase2 = [];
stepTimes.unexp.phase3 = [];

figure
for ph = 1:length(phs)
    subplot(1,6,ph); hold on; grid on;
    for exp = [1 2 3 4 5] 
%         time = db.(exps{exp}).(phs{ph}).timeMax;
        time = timeEvents.(exps{exp}).(phs{ph}).timeMax;
        plot(downsteps(exp),time,strcat(colors{exp},'o'))
        stepTimes.exp.(phs{ph}) = [stepTimes.exp.(phs{ph}); time];
    end
    title('expected')
    legend('exp00','exp25','exp50','exp75','exp100')
    
    subplot(1,6,ph+3); hold on; grid on;
    for exp = [1 6 7 8 9]
%         time = db.(exps{exp}).(phs{ph}).timeMax;
        time = timeEvents.(exps{exp}).(phs{ph}).timeMax;
        plot(downsteps(exp),time,strcat(colors{exp},'o'))
        stepTimes.unexp.(phs{ph}) = [stepTimes.unexp.(phs{ph}); time];
    end
    title('unexpected')
    legend('unexp00','unexp25','unexp50','unexp75','unexp100')
end

stepTimes.exp.Ts1 = polyfit(downsteps(1:5),stepTimes.exp.phase1,1);
stepTimes.exp.Ts2 = polyfit(downsteps(1:5),stepTimes.exp.phase2,1);
stepTimes.exp.Ts3 = polyfit(downsteps(1:5),stepTimes.exp.phase3,1);
stepTimes.unexp.Ts1 = polyfit(downsteps(1:5),stepTimes.unexp.phase1,1);
stepTimes.unexp.Ts2 = polyfit(downsteps(1:5),stepTimes.unexp.phase2,1);
stepTimes.unexp.Ts3 = polyfit(downsteps(1:5),stepTimes.unexp.phase3,1);


subplot(1,6,1); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.exp.Ts1,heval))
ylabel('Steptime')
xlabel('downstep [cm]')

subplot(1,6,2); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.exp.Ts2,heval))
xlabel('downstep [cm]')

subplot(1,6,3); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.exp.Ts3,heval))
xlabel('downstep [cm]')

subplot(1,6,4); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.unexp.Ts1,heval))
ylabel('Steptime')
xlabel('downstep [cm]')

subplot(1,6,5); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.unexp.Ts2,heval))
xlabel('downstep [cm]')

subplot(1,6,6); hold on; grid on;
heval = -0.10:0.01:0;
plot(heval,polyval(stepTimes.unexp.Ts3,heval))
xlabel('downstep [cm]')

linkaxes

bezierStepTimes = stepTimes;
save('data/human/bezierStepTimes.mat','bezierStepTimes')
save('data/outputs/bezierStepTimes.mat','bezierStepTimes')

%% Bezier scaling
bvFits = struct;
bvFits.exp = struct;
bvFits.unexp = struct;

for ph = 1:length(phs)
    % expected
    for ii = 1:length(ab.exp00.(phs{ph}).bv_zcom) % num of Bezier parameters
        bv_param = [];
        for exp = [1 2 3 4 5] % [1 2 3 4 5]
            bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_zcom(ii)];
        end
        bvFits.exp.(phs{ph}).zcom{ii} = polyfit(downsteps(1:5),bv_param,1);
    end
    for ii = 1:length(ab.exp00.(phs{ph}).bv_dzcom) % num of Bezier parameters
        bv_dparam = [];
        for exp = [1 2 3 4 5] % [1 2 3 4 5]
            bv_dparam = [bv_dparam ab.(exps{exp}).(phs{ph}).bv_dzcom(ii)];
        end
        bvFits.exp.(phs{ph}).dzcom{ii} = polyfit(downsteps(1:5),bv_dparam,1);
    end
    
    % unexpected
    for ii = 1:length(ab.exp00.(phs{ph}).bv_zcom) % num of Bezier parameters
        bv_param = [];
        for exp = [1 6 7 8 9] % [1 2 3 4 5]
            bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_zcom(ii)];
        end
        bvFits.unexp.(phs{ph}).zcom{ii} = polyfit(downsteps(1:5),bv_param,1);
    end
    for ii = 1:length(ab.exp00.(phs{ph}).bv_dzcom) % num of Bezier parameters
        bv_dparam = [];
        for exp = [1 2 3 4 5] % [1 2 3 4 5]
            bv_dparam = [bv_dparam ab.(exps{exp}).(phs{ph}).bv_dzcom(ii)];
        end
        bvFits.unexp.(phs{ph}).dzcom{ii} = polyfit(downsteps(1:5),bv_dparam,1);
    end
end

bezierZcomInterpolation = bvFits;
save('data/outputs/bezierZcomInterpolation.mat','bezierZcomInterpolation')


%% Plot zcom for downsteps
hs = linspace(0,-0.30,30);

figure
subplot(2,3,1); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts1,h),100);
    plot(time,bezier2(bv,teval))
end
xlabel('time')
ylabel('zcom')
title('expected')

subplot(2,3,2); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts2,h),100);
    plot(time,bezier2(bv,teval))
end
title('expected')

subplot(2,3,3); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts3,h),100);
    plot(time,bezier2(bv,teval))
end
title('expected')




subplot(2,3,4); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
    plot(time,bezier2(bv,teval))
end
xlabel('time')
ylabel('zcom')
title('unexpected')

subplot(2,3,5); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts2,h),100);
    plot(time,bezier2(bv,teval))
end
title('unexpected')

subplot(2,3,6); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts3,h),100);
    plot(time,bezier2(bv,teval))
end
title('unexpected')

linkaxes

%% Plotting combined
figure
subplot(1,3,1); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts1,h),100);
    plot(time,bezier2(bv,teval),'r')
    
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
    plot(time,bezier2(bv,teval),'b')
    
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')

subplot(1,3,2); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts2,h),100);
    plot(time,bezier2(bv,teval),'r')
    
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts2,h),100);
    plot(time,bezier2(bv,teval),'b')
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')

subplot(1,3,3); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.exp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts3,h),100);
    plot(time,bezier2(bv,teval),'r')
    
    bv = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts3,h),100);
    plot(time,bezier2(bv,teval),'b')
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')



