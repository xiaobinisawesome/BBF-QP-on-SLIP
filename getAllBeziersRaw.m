clearvars; close all; clc;

%% Generate downstep bezier curves
addpath('functions')

exps = {'exp00','exp25','exp50','exp75','exp100',...
        'unexp25','unexp50','unexp75','unexp100'};
stepHeights = [0.00 0.025 0.050 0.075 0.100 0.025 0.050 0.075 0.100];

%%%%%%%%%%%%
%%% get data
allDataHuman = struct;
allDataCassie = struct;
figure(101); hold on; grid on;
for i = 1:length(exps)
    [allDataHuman,allDataCassie] = orderHumanData(exps{i},stepHeights(i),allDataHuman,allDataCassie);
end
allData = allDataHuman;
save('data/beziersRaw/allDataHuman.mat','allData')
allData = allDataCassie;
save('data/beziersRaw/allDataCassie.mat','allData')

%%%%%%%%%%%%%%%
%%% get beziers
allBeziersHuman = struct;
allBeziersCassie = struct;
figure(202); hold on; grid on;
figure(303); hold on; grid on;
figure(404); hold on; grid on;

    allBeziersHuman = fitHumanDataToBezier(allDataHuman,allBeziersHuman);
    allBeziersCassie = fitHumanDataToBezier(allDataCassie,allBeziersCassie);

allBeziers = allBeziersHuman;
save('data/beziersRaw/allBeziersHuman.mat','allBeziers')
allBeziers = allBeziersCassie;
save('data/beziersRaw/allBeziersCassie.mat','allBeziers')


%% FUNCTIONS
function allBeziersHuman = fitHumanDataToBezier(allDataHuman,allBeziersHuman)
filename = strcat('data/human/timeEvents.mat');
load(filename)

fns = fieldnames(allDataHuman);
for i = 1:length(fns) % exp00, exp25, exp50, ...
    fns2 = fieldnames(allDataHuman.(fns{i}));
    for ii = 1:length(fns2) % phase1, phase2, phase3
        time = allDataHuman.(fns{i}).(fns2{ii}).time;
        time_SSP = allDataHuman.(fns{i}).(fns2{ii}).time_SSP;
        time_norm = allDataHuman.(fns{i}).(fns2{ii}).time_norm;
        time_norm_SSP = allDataHuman.(fns{i}).(fns2{ii}).time_norm_SSP;
        
        zcom = allDataHuman.(fns{i}).(fns2{ii}).zcom;
        dzcom = allDataHuman.(fns{i}).(fns2{ii}).dzcom;
        ddzcom = allDataHuman.(fns{i}).(fns2{ii}).ddzcom;
        
        xcom = allDataHuman.(fns{i}).(fns2{ii}).xcom;
        dxcom = allDataHuman.(fns{i}).(fns2{ii}).dxcom;
        ddxcom = allDataHuman.(fns{i}).(fns2{ii}).ddxcom;

        %%%%%%%%%%%%%
        % ZCOM
        mid = round(length(zcom)/2);
        
        if ii == 1
            z0 = 0.89;
            bv0 = [zcom(1) zcom(1) zcom(end) zcom(end)];
            fun = @(x) costFcnBezierCustom(x,time_norm,zcom,[z0 z0],[]);
            bv_zcom = fmincon(fun,bv0,[],[],[],[])
            bv_zcom = [z0 z0 bv_zcom];
            
            tMax = timeEvents.(fns{i}).(fns2{ii}).timeMax;  
            figure(202); 
            subplot(1,2,1); hold on; grid on;
            plot(linspace(0,tMax,100),bezier2(bv_zcom,linspace(0,1,100)))
            subplot(1,2,2); hold on; grid on;
            plot(time,zcom)
        elseif ii == 3
            z0 = 0.89;
            bv0 = [zcom(end)];
            fun = @(x) costFcnBezierCustom(x,time_norm,zcom,[zcom(1)],[z0 z0]);
            bv_zcom = fmincon(fun,bv0,[],[],[],[])
            bv_zcom = [zcom(1) bv_zcom z0 z0];
            
            tMax = timeEvents.(fns{i}).(fns2{ii}).timeMax;  
            figure(303); 
            subplot(1,2,1); hold on; grid on;
            plot(linspace(0,tMax,100),bezier2(bv_zcom,linspace(0,1,100)))
            subplot(1,2,2); hold on; grid on;
            plot(time,zcom)
        else
            bv0 = [zcom(1) zcom(1) zcom(end)];
            fun = @(x) costFcnBezierCustom(x,time_norm,zcom,[zcom(1)],[zcom(end)]);
            bv_zcom = fmincon(fun,bv0,[],[],[],[])
            bv_zcom = [zcom(1) bv_zcom zcom(end)];
            
            tMax = timeEvents.(fns{i}).(fns2{ii}).timeMax;  
            figure(404); 
            subplot(1,2,1); hold on; grid on;
            plot(linspace(0,tMax,100),bezier2(bv_zcom,linspace(0,1,100)))
            subplot(1,2,2); hold on; grid on;
            plot(time,zcom)
        end
        
        bv0 = [dzcom(1) dzcom(1) dzcom(mid) dzcom(end) dzcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,dzcom);
        bv_dzcom = fmincon(fun,bv0,[],[],[],[])

        bv0 = [ddzcom(1) ddzcom(1) ddzcom(mid) ddzcom(end) ddzcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,ddzcom);
        bv_ddzcom = fmincon(fun,bv0,[],[],[],[])

        %%%%%%%%%%%%%
        % XCOM
        bv0 = [xcom(1) xcom(1) xcom(mid) xcom(end) xcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,xcom);
        bv_xcom = fmincon(fun,bv0,[],[],[],[])

        bv0 = [dxcom(1) dxcom(1) dxcom(mid) dxcom(end) dxcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,dxcom);
        bv_dxcom = fmincon(fun,bv0,[],[],[],[])

        bv0 = [ddxcom(1) ddxcom(1) ddxcom(mid) ddxcom(end) ddxcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,ddxcom);
        bv_ddxcom = fmincon(fun,bv0,[],[],[],[])
        
        % to struct
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_zcom = bv_zcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_dzcom = bv_dzcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_ddzcom = bv_ddzcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_xcom = bv_xcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_dxcom = bv_dxcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).bv_ddxcom = bv_ddxcom;
        allBeziersHuman.(fns{i}).(fns2{ii}).timeMax = time(end);
        allBeziersHuman.(fns{i}).(fns2{ii}).timeMax_SSP = time_SSP(end);
    end
end
end


function [allDataHuman,allDataCassie] = orderHumanData(exp,stepHeight,allDataHuman,allDataCassie)
filename = strcat('data/human/humanDataCoM.mat');
load(filename)

filename = strcat('data/human/timeEvents.mat');
load(filename)

downstep = stepHeight;

idx1 = 1:timeEvents.(exp).rawIdx(3);
idx1SSP = 1:timeEvents.(exp).rawIdx(2);

idx2 = timeEvents.(exp).rawIdx(3):timeEvents.(exp).rawIdx(6);
idx2SSP = timeEvents.(exp).rawIdx(3):timeEvents.(exp).rawIdx(5);

idx3 = timeEvents.(exp).rawIdx(6):timeEvents.(exp).rawIdx(7);
idx3SSP = timeEvents.(exp).rawIdx(6):timeEvents.(exp).rawIdx(7);

%% timing
time_human_1 = humanDataCoM.(exp).t(idx1) - humanDataCoM.(exp).t(idx1(1));
time_human_SSP_1 = humanDataCoM.(exp).t(idx1SSP) - humanDataCoM.(exp).t(idx1SSP(1));
time_norm_1 = time_human_1./(time_human_1(end));
time_norm_SSP_1 = time_human_SSP_1./(time_human_SSP_1(end));

time_human_2 = humanDataCoM.(exp).t(idx2) - humanDataCoM.(exp).t(idx2(1));
time_human_SSP_2 = humanDataCoM.(exp).t(idx2SSP) - humanDataCoM.(exp).t(idx2SSP(1));
time_norm_2 = time_human_2./(time_human_2(end));
time_norm_SSP_2 = time_human_SSP_2./(time_human_SSP_2(end));

time_human_3 = humanDataCoM.(exp).t(idx3) - humanDataCoM.(exp).t(idx3(1));
time_human_SSP_3 = humanDataCoM.(exp).t(idx3SSP) - humanDataCoM.(exp).t(idx3SSP(1));
time_norm_3 = time_human_3./(time_human_3(end));
time_norm_SSP_3 = time_human_SSP_3./(time_human_SSP_3(end));

%% Outputs
% step 1
xcom_human_1 = humanDataCoM.(exp).x(idx1);
dxcom_human_1 = humanDataCoM.(exp).dx(idx1);
ddxcom_human_1 = humanDataCoM.(exp).ddx(idx1);

zcom_human_1 = humanDataCoM.(exp).y(idx1);
dzcom_human_1 = humanDataCoM.(exp).dy(idx1);
ddzcom_human_1 = humanDataCoM.(exp).ddy(idx1);

% step 2
xcom_human_2 = humanDataCoM.(exp).x(idx2);
dxcom_human_2 = humanDataCoM.(exp).dx(idx2);
ddxcom_human_2 = humanDataCoM.(exp).ddx(idx2);

zcom_human_2 = humanDataCoM.(exp).y(idx2);
dzcom_human_2 = humanDataCoM.(exp).dy(idx2);
ddzcom_human_2 = humanDataCoM.(exp).ddy(idx2);

% step 3
xcom_human_3 = humanDataCoM.(exp).x(idx3);
dxcom_human_3 = humanDataCoM.(exp).dx(idx3);
ddxcom_human_3 = humanDataCoM.(exp).ddx(idx3);

zcom_human_3 = humanDataCoM.(exp).y(idx3);
dzcom_human_3 = humanDataCoM.(exp).dy(idx3);
ddzcom_human_3 = humanDataCoM.(exp).ddy(idx3);

figure(101);
plot(time_human_1,zcom_human_1)
% figure(10101);
% if any(strcmp(exp,{'exp00','unexp25','unexp50','unexp75','unexp100'}))
%     plot(time_human_1,zcom_human_1)
% end

%% Save all data as struct
allDataHuman.(exp).phase1.time = time_human_1;
allDataHuman.(exp).phase1.time_SSP = time_human_SSP_1;
allDataHuman.(exp).phase1.time_norm = time_norm_1;
allDataHuman.(exp).phase1.time_norm_SSP = time_norm_SSP_1;
allDataHuman.(exp).phase1.xcom = xcom_human_1;
allDataHuman.(exp).phase1.dxcom = dxcom_human_1;
allDataHuman.(exp).phase1.ddxcom = ddxcom_human_1;
allDataHuman.(exp).phase1.zcom = zcom_human_1;
allDataHuman.(exp).phase1.dzcom = dzcom_human_1;
allDataHuman.(exp).phase1.ddzcom = ddzcom_human_1;

allDataHuman.(exp).phase2.time = time_human_2;
allDataHuman.(exp).phase2.time_SSP = time_human_SSP_2;
allDataHuman.(exp).phase2.time_norm = time_norm_2;
allDataHuman.(exp).phase2.xcom = xcom_human_2;
allDataHuman.(exp).phase2.time_norm_SSP = time_norm_SSP_2;
allDataHuman.(exp).phase2.dxcom = dxcom_human_2;
allDataHuman.(exp).phase2.ddxcom = ddxcom_human_2;
allDataHuman.(exp).phase2.zcom = zcom_human_2;
allDataHuman.(exp).phase2.dzcom = dzcom_human_2;
allDataHuman.(exp).phase2.ddzcom = ddzcom_human_2;

allDataHuman.(exp).phase3.time = time_human_3;
allDataHuman.(exp).phase3.time_SSP = time_human_SSP_3;
allDataHuman.(exp).phase3.time_norm = time_norm_3;
allDataHuman.(exp).phase3.time_norm_SSP = time_norm_SSP_3;
allDataHuman.(exp).phase3.xcom = xcom_human_3;
allDataHuman.(exp).phase3.dxcom = dxcom_human_3;
allDataHuman.(exp).phase3.ddxcom = ddxcom_human_3;
allDataHuman.(exp).phase3.zcom = zcom_human_3;
allDataHuman.(exp).phase3.dzcom = dzcom_human_3;
allDataHuman.(exp).phase3.ddzcom = ddzcom_human_3;

%% Scaling
load('data/beziers/scaling.mat')
stepLength_human = scaling.stepLength_human;
stepLength_cassie = scaling.stepLength_cassie;
stepLength_scaling = scaling.stepLength_scaling;
x_scaling = scaling.x_scaling;
dx_scaling = scaling.dx_scaling;
ddx_scaling = scaling.ddx_scaling;
z_scaling = scaling.z_scaling;
dz_scaling = scaling.dz_scaling;
ddz_scaling = scaling.ddz_scaling;
grf_scaling = scaling.grf_scaling;
Ts_scaling = scaling.Ts_scaling;
L_scaling = scaling.L_scaling;

strAdd = {'1','2','3'};
for i = 1:3
    add = strAdd{i};
    time_cassie = Ts_scaling*eval(strcat('time_human_',add));
    time_cassie_SSP = Ts_scaling*eval(strcat('time_human_SSP_',add));
    time_norm = eval(strcat('time_norm_',add));
    time_norm_SSP = eval(strcat('time_norm_SSP_',add));
    
    xcom_cassie = x_scaling*eval(strcat('xcom_human_',add));
    dxcom_cassie = dx_scaling*eval(strcat('dxcom_human_',add));
    ddxcom_cassie = ddx_scaling*eval(strcat('ddxcom_human_',add));
    
    L_human = sqrt(eval(strcat('xcom_human_',add)).^2 + ...
                   eval(strcat('zcom_human_',add)).^2);
    L_cassie = L_scaling*L_human;
    zcom_cassie = sqrt(L_cassie.^2 - xcom_cassie.^2);
    dzcom_cassie = dz_scaling*eval(strcat('dzcom_human_',add));
    ddzcom_cassie = ddz_scaling*eval(strcat('ddzcom_human_',add));

    
    allDataCassie.(exp).(strcat('phase',add)).time = time_cassie;
    allDataCassie.(exp).(strcat('phase',add)).time_SSP = time_cassie_SSP;
    allDataCassie.(exp).(strcat('phase',add)).time_norm = time_norm;
    allDataCassie.(exp).(strcat('phase',add)).time_norm_SSP = time_norm_SSP;
    allDataCassie.(exp).(strcat('phase',add)).xcom = xcom_cassie;
    allDataCassie.(exp).(strcat('phase',add)).dxcom = dxcom_cassie;
    allDataCassie.(exp).(strcat('phase',add)).ddxcom = ddxcom_cassie;
    allDataCassie.(exp).(strcat('phase',add)).zcom = zcom_cassie;
    allDataCassie.(exp).(strcat('phase',add)).dzcom = dzcom_cassie;
    allDataCassie.(exp).(strcat('phase',add)).ddzcom = ddzcom_cassie;    
end

end





%% FUNCTIONS
function printBezier(fid,data,name)
    fprintf(fid,'this->%s.resize(%i);\n',name,length(data));
    fprintf(fid,'this->%s << %f,\n',name,data(1));
    for i = 2:length(data)-1
        fprintf(fid,'%f,\n',data(i));
    end
    fprintf(fid,'%f;\n\n\n',data(end));
end
