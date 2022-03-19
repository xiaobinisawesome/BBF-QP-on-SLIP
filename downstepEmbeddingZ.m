clear all; close all; clc;
addpath('functions/')

%% Load data
system = 'Cassie';
load(strcat('data/beziersRaw/allBeziers',system,'.mat'))
ab = allBeziers;

%% Some settings for later down the line
colors = {'r', 'b', 'g', 'c', 'k', ...
          'r', 'b', 'g', 'c', 'k'};
downsteps = [0 -0.025 -0.05 -0.075 -0.10 ...
             0 -0.025 -0.05 -0.075 -0.10];
exps = fieldnames(ab);
phs = fieldnames(ab.exp00);
teval = linspace(0,1,100);

%% step time loading and scaling if necessary
load('data/human/bezierStepTimes.mat')
load('data/beziersRaw/scaling.mat')
if isequal(system,'Cassie')
    for exp = {'exp','unexp'}
        bezierStepTimes.(exp{:}).phase1 = scaling.Ts_scaling*bezierStepTimes.(exp{:}).phase1;
        bezierStepTimes.(exp{:}).phase2 = scaling.Ts_scaling*bezierStepTimes.(exp{:}).phase2;
        bezierStepTimes.(exp{:}).phase3 = scaling.Ts_scaling*bezierStepTimes.(exp{:}).phase3;
        
        bezierStepTimes.(exp{:}).Ts1(2) = scaling.Ts_scaling*bezierStepTimes.(exp{:}).Ts1(2);
        bezierStepTimes.(exp{:}).Ts2(2) = scaling.Ts_scaling*bezierStepTimes.(exp{:}).Ts2(2);
        bezierStepTimes.(exp{:}).Ts3(2) = scaling.Ts_scaling*bezierStepTimes.(exp{:}).Ts3(2);
    end
end


load('data/human/timeEvents.mat')
if isequal(system,'Cassie')
    for i = 1:length(exps)
        exp = exps{i};
        timeEvents.(exp).phase1.timeMax = scaling.Ts_scaling*timeEvents.(exp).phase1.timeMax;
        timeEvents.(exp).phase1.timeMax_SSP = scaling.Ts_scaling*timeEvents.(exp).phase1.timeMax_SSP;
        timeEvents.(exp).phase2.timeMax = scaling.Ts_scaling*timeEvents.(exp).phase2.timeMax;
        timeEvents.(exp).phase2.timeMax_SSP = scaling.Ts_scaling*timeEvents.(exp).phase2.timeMax_SSP;
        timeEvents.(exp).phase3.timeMax = scaling.Ts_scaling*timeEvents.(exp).phase3.timeMax;
        timeEvents.(exp).phase3.timeMax_SSP = scaling.Ts_scaling*timeEvents.(exp).phase3.timeMax_SSP;
    end
end


%% Set first two indices of zcom equal
ab.exp00.phase1.bv_zcom(2) = ab.exp00.phase1.bv_zcom(1);
for exp = 1:length(exps)
    ab.(exps{exp}).('phase1').bv_zcom(1:2) = ab.exp00.('phase1').bv_zcom(1:2);
    ab.(exps{exp}).('phase3').bv_zcom(4:5) = ab.exp00.('phase3').bv_zcom(4:5);
end

%% Some initial plotting
% figure
% for ph = 1:length(phs)
%     subplot(2,6,ph); hold on; grid on;
%     for exp = [1 2 3 4 5]
% %         timeMax = db.(exps{exp}).(phs{ph}).timeMax;
%         timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax;
%         time = linspace(0,timeMax,100);
%         plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
%     end
%     xlabel('time')
%     title('expected')
%     
%     subplot(2,6,ph+6); hold on; grid on;
%     for exp = [1 2 3 4 5]
%         xcom = bezier2(ab.(exps{exp}).(phs{ph}).bv_xcom,teval);
%         plot(xcom,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
%     end
%     xlabel('xcom')
%     legend('exp00','exp25','exp50','exp75','exp100')
%     
%     subplot(2,6,ph+3); hold on; grid on;
%     for exp = [6 7 8 9 10]
% %         timeMax = db.(exps{exp}).(phs{ph}).timeMax;
%         timeMax = timeEvents.(exps{exp}).(phs{ph}).timeMax;
%         time = linspace(0,timeMax,100);
%         plot(time,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
%     end
%     xlabel('time')
%     title('unexpected')
%     
%     subplot(2,6,ph+9); hold on; grid on;
%     for exp = [6 7 8 9 10]
%         xcom = bezier2(ab.(exps{exp}).(phs{ph}).bv_xcom,teval);
%         plot(xcom,bezier2(ab.(exps{exp}).(phs{ph}).bv_zcom,teval))
%     end
%     xlabel('xcom')
%     legend('unexp00','unexp25','unexp50','unexp75','unexp100')
% end


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
    for exp = [6 7 8 9 10]
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
save(strcat('data/outputs/bezierStepTimes',system,'.mat'),'bezierStepTimes')

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
        for exp = [6 7 8 9 10] % [1 2 3 4 5]
            bv_param = [bv_param ab.(exps{exp}).(phs{ph}).bv_zcom(ii)];
        end
        bvFits.unexp.(phs{ph}).zcom{ii} = polyfit(downsteps(1:5),bv_param,1);
    end
    for ii = 1:length(ab.exp00.(phs{ph}).bv_dzcom) % num of Bezier parameters
        bv_dparam = [];
        for exp = [6 7 8 9 10] % [1 2 3 4 5]
            bv_dparam = [bv_dparam ab.(exps{exp}).(phs{ph}).bv_dzcom(ii)];
        end
        bvFits.unexp.(phs{ph}).dzcom{ii} = polyfit(downsteps(1:5),bv_dparam,1);
    end
end


%% Plot zcom for downsteps
hs = linspace(0,-0.30,30);

figure
subplot(2,3,1); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts1,h),100);
    plot(time,bezier2(bv_1,teval))
end
xlabel('time')
ylabel('zcom')
title('expected')

subplot(2,3,2); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts2,h),100);
    plot(time,bezier2(bv_1,teval))
end
title('expected')

subplot(2,3,3); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts3,h),100);
    plot(time,bezier2(bv_1,teval))
end
title('expected')




subplot(2,3,4); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
    plot(time,bezier2(bv_1,teval))
end
xlabel('time')
ylabel('zcom')
title('unexpected')

subplot(2,3,5); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts2,h),100);
    plot(time,bezier2(bv_1,teval))
end
title('unexpected')

subplot(2,3,6); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts3,h),100);
    plot(time,bezier2(bv_1,teval))
end
title('unexpected')

linkaxes

%% Plotting combined
figure
subplot(1,3,1); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase1.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts1,h),100);
    plot(time,bezier2(bv_1,teval),'r')
    
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
    plot(time,bezier2(bv_1,teval),'b')
    
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')

subplot(1,3,2); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase2.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts2,h),100);
    plot(time,bezier2(bv_1,teval),'r')
    
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts2,h),100);
    plot(time,bezier2(bv_1,teval),'b')
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')

subplot(1,3,3); hold on; grid on;
for h = hs
    n = length(ab.exp00.phase3.bv_zcom);
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.exp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.exp.Ts3,h),100);
    plot(time,bezier2(bv_1,teval),'r')
    
    bv_1 = zeros(1,n);
    % get the bezier poly's for phase 1
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
    end
    
    time = linspace(0,polyval(stepTimes.unexp.Ts3,h),100);
    plot(time,bezier2(bv_1,teval),'b')
end
legend('expected','unexpected')
xlabel('time')
ylabel('zcom')


%% plot h=0 unexpected
h = 0.0;

figure
subplot(1,3,1); hold on; grid on;
bv_1 = zeros(1,n);
for ii = 1:n
    bv_1(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
end
time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
plot(time,bezier2(bv_1,teval),'b')

subplot(1,3,2); hold on; grid on;
bv_2 = zeros(1,n);
for ii = 1:n
    bv_2(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
end
time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
plot(time,bezier2(bv_2,teval),'b')

subplot(1,3,3); hold on; grid on;
bv_3 = zeros(1,n);
for ii = 1:n
    bv_3(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
end
time = linspace(0,polyval(stepTimes.unexp.Ts1,h),100);
plot(time,bezier2(bv_3,teval),'b')

linkaxes

% fit a bezier starting at VLO and use this for nominal walking trajectory
time = 0.3024 + 0.1130;
zcom1 = bezier2(bv_1,teval);
zcom2 = bezier2(bv_2,teval);

[~,idx] = max(zcom2);
zcom2 = zcom2(1:idx);

zData = [zcom1 zcom2(2:end)];
tData = [linspace(0,0.3024,100) 0.3024+0.0031+linspace(0,0.1130,idx-1)];

figure
plot(tData,zData)

fun = @(bv) rms(bezier2(bv,linspace(0,0.3024/time)) - zcom1);
Aeq = [1 0 0 0 0 0 -1;
       1 -1 0 0 0 0 0;
       0 0 0 0 0 1 -1;
       1 0 0 0 0 0 0];
beq = [0;
       0;
       0
       zcom1(1)];
   
bv0 = [zcom1(1) zcom1(1) zcom1(end/2) zcom1(end/2) zcom1(end/2) zcom2(end) zcom2(end)];
bv_zcom = fmincon(fun,bv0,[],[],Aeq,beq,[],[]);

figure; 
subplot(1,2,1); hold on; grid on;
plot(teval*time,bezier2(bv_zcom,teval))
plot(tData,zData)
subplot(1,2,2); hold on; grid on;
plot(teval*time,bezier2(bv_zcom,teval,1))

% fit a bezier starting at end of DSP and use this for nominal walking trajectory
time = 0.3024 + 0.1130;
zcom1 = bezier2(bv_1,teval);
zcom2 = bezier2(bv_2,teval);

[~,idx] = max(zcom2);
zcom2 = zcom2(1:idx);

zData = [zcom2-(0.8964-0.8915) zcom1(2:end)];
tData = [linspace(0,0.1130,idx) 0.1130+0.0031+linspace(0,0.3024,99)];

figure
plot(tData,zData)

fun = @(bv) rms(bezier2(bv,linspace(0.1161/time,1,100)) - zcom1);
Aeq = [1 0 0 0 0 0 -1];
beq = [0];
   
nlcon = @(bv) deal([],[bezier2(bv,0,1) - bezier2(bv,1,1);
                       bezier2(bv,0.1161/time) - max(zData)]);
                   
bv0 = [zData(1) zData(1) zData(round(end/2)) zData(round(end/2)) zData(round(end/2)) zData(end) zData(end)];
bv_zcom = fmincon(fun,bv0,[],[],Aeq,beq,[],[],nlcon);

figure; 
subplot(1,2,1); hold on; grid on;
plot(teval*time,bezier2(bv_zcom,teval))
plot(tData,zData)
legend('bezier','data')
subplot(1,2,2); hold on; grid on;
plot(teval*time,bezier2(bv_zcom,teval,1))


bvFits.exp00.bv_zcom_VLO = bv_zcom;
bvFits.exp00.bv_zcom = bv_zcom;

%% SAVING STRUCT
bezierZcomInterpolation = bvFits;
save(strcat('data/outputs/bezierZcomInterpolation',system,'.mat'),'bezierZcomInterpolation')





