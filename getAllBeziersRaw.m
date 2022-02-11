clearvars; 
% close all; 
clc;

%% Generate downstep bezier curves
addpath('functions')

exps = {'exp00','exp25','exp50','exp75','exp100',...
        'unexp00','unexp25','unexp50','unexp75','unexp100'};
stepHeights = [0.00 0.025 0.050 0.075 0.100 ...
               0.00 0.025 0.050 0.075 0.100];

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

    allBeziersHuman = fitDataToBezier(allDataHuman,allBeziersHuman);
%     allBeziersCassie = fitDataToBezier(allDataCassie,allBeziersCassie);

allBeziers = allBeziersHuman;
save('data/beziersRaw/allBeziersHuman.mat','allBeziers')
allBeziers = allBeziersCassie;
save('data/beziersRaw/allBeziersCassie.mat','allBeziers')


%% FUNCTIONS
function allBeziers = fitDataToBezier(allData,allBeziers)
filename = strcat('data/human/timeEvents.mat');
load(filename)

fns = fieldnames(allData);
for fn = 1:length(fns) % exp00, exp25, exp50, ...
    phs = fieldnames(allData.(fns{fn}));
    for ph = 1:length(phs) % phase1, phase2, phase3
        time = allData.(fns{fn}).(phs{ph}).time;
        time_SSP = allData.(fns{fn}).(phs{ph}).time_SSP;
        time_norm = allData.(fns{fn}).(phs{ph}).time_norm;
        time_norm_SSP = allData.(fns{fn}).(phs{ph}).time_norm_SSP;
        
        zcom = allData.(fns{fn}).(phs{ph}).zcom;
        dzcom = allData.(fns{fn}).(phs{ph}).dzcom;
        ddzcom = allData.(fns{fn}).(phs{ph}).ddzcom;
        
        xcom = allData.(fns{fn}).(phs{ph}).xcom;
        dxcom = allData.(fns{fn}).(phs{ph}).dxcom;
        ddxcom = allData.(fns{fn}).(phs{ph}).ddxcom;

        %%%%%%%%%%%%%
        %% ZCOM        
        if ph == 1
            zmax = max(allData.exp00.('phase1').zcom);
            
            zcom1 = zcom;
            zcom2 = allData.(fns{fn}).('phase2').zcom;
            zcom3 = allData.(fns{fn}).('phase3').zcom;
            
            scaleTowardsNominal = true;
            if scaleTowardsNominal && fn >= 6 % unexpected phases only!
                delta2 = 0.8905 - 0.8795;
                scaling = linspace(1,(zcom3(end)-delta2 - (zcom3(end)-zmax))/zcom3(end),length(zcom3))';
                zcom3 = zcom3.*scaling;
                zmaxEnd = zmax - delta2;
            else
                scaling = linspace(1,(zcom3(end) - (zcom3(end)-zmax))/zcom3(end),length(zcom3))';
                zcom3 = zcom3.*scaling;
                zmaxEnd = zmax;
            end
            
            time_norm1 = time_norm;
            time_norm2 = allData.(fns{fn}).('phase2').time_norm;
            time_norm3 = allData.(fns{fn}).('phase3').time_norm;
            time_human1 = time;
            time_human2 = allData.(fns{fn}).('phase2').time;
            time_human3 = allData.(fns{fn}).('phase3').time;
            
            bv0 = [zcom1(1) zcom1(1) zcom1(round(end/2)) zcom1(end) zcom1(end) zcom1(end) ...
                   zcom2(1) zcom2(1) zcom2(round(end/2)) zcom2(end) zcom2(end) zcom2(end) ...
                   zcom3(1) zcom3(1) zcom3(round(end/2)) zcom3(end) zcom3(end) zcom3(end)];
            fun = @(x) costFcnBezierZ(x,time_norm1,time_norm2,time_norm3,...
                                        zcom1,zcom2,zcom3,...
                                        6,6,6);
            Aeq = [1 -1 0 0 0 0 zeros(1,12);
                   zeros(1,12) 0 0 0 0 1 -1;
                   0 0 0 0 0 1 -1 0 0 0 0 0 zeros(1,6);
                   zeros(1,6) 0 0 0 0 0 1 -1 0 0 0 0 0;
                   1 0 0 0 0 0 zeros(1,12);
                   zeros(1,12) 0 0 0 0 0 1];
            beq = [zeros(4,1);
                   zmax;
                   zmaxEnd];
            Aneq = [];
            bneq = [];
            
            n1 = length(time_human1);
            n2 = length(time_human2);
            n3 = length(time_human3);
            nlcon = @(bv) deal([],[bezier2(bv(1:6),1,1)/n1 - bezier2(bv(7:12),0,1)/n2;
                                   bezier2(bv(7:12),1,1)/n2 - bezier2(bv(13:18),0,1)/n3]);
            bv_zcom_all = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            bv_zcom_phase1 = bv_zcom_all(1:6);
            bv_zcom_phase2 = bv_zcom_all(7:12);
            bv_zcom_phase3 = bv_zcom_all(13:18);
            
            teval = 0:0.01:1;
            figure; 
            subplot(2,3,1); hold on; grid on;
            plot(teval,bezier2(bv_zcom_phase1,teval),'r')
            plot(linspace(0,1,length(zcom1)),zcom1,'b')
            subplot(2,3,2); hold on; grid on;
            plot(teval,bezier2(bv_zcom_phase2,teval),'r')
            plot(linspace(0,1,length(zcom2)),zcom2,'b')
            subplot(2,3,3); hold on; grid on;
            plot(teval,bezier2(bv_zcom_phase3,teval),'r')
            plot(linspace(0,1,length(zcom3)),zcom3,'b')
            legend('bezier','raw')
            
            subplot(2,3,[4 5 6]); hold on; grid on;
            plot(time_human1,bezier2(bv_zcom_phase1,time_norm1),'r')
            plot(time_human1,zcom1,'b')
            plot(time_human2+time_human1(end),bezier2(bv_zcom_phase2,time_norm2),'r')
            plot(time_human2+time_human1(end),zcom2,'b')
            plot(time_human3+time_human1(end)+time_human2(end),bezier2(bv_zcom_phase3,time_norm3),'r')
            plot(time_human3+time_human1(end)+time_human2(end),zcom3,'b')
            linkaxes
            sgtitle(phs{ph})
            
            allBeziers.(fns{fn}).('phase1').bv_zcom = bv_zcom_phase1;
            allBeziers.(fns{fn}).('phase2').bv_zcom = bv_zcom_phase2;
            allBeziers.(fns{fn}).('phase3').bv_zcom = bv_zcom_phase3;
        end
        
        if ph == 1
            n = 5;
            dzcom1 = dzcom;
            dzcom2 = allData.(fns{fn}).('phase2').dzcom;
            dzcom3 = allData.(fns{fn}).('phase3').dzcom;
            time_norm1 = time_norm;
            time_norm2 = allData.(fns{fn}).('phase2').time_norm;
            time_norm3 = allData.(fns{fn}).('phase3').time_norm;
            time_human1 = time;
            time_human2 = allData.(fns{fn}).('phase2').time;
            time_human3 = allData.(fns{fn}).('phase3').time;
            
            bv0 = [dzcom1(1) dzcom1(1) dzcom1(round(end/2)) dzcom1(end) dzcom1(end) ...
                   dzcom2(1) dzcom2(1) dzcom2(round(end/2)) dzcom2(end) dzcom2(end) ...
                   dzcom3(1) dzcom3(1) dzcom3(round(end/2)) dzcom3(end) dzcom3(end)];
            fun = @(x) costFcnBezierZ(x,time_norm1,time_norm2,time_norm3,...
                                        dzcom1,dzcom2,dzcom3,...
                                        n,n,n);
            Aeq = [0 0 0 0 1 -1 0 0 0 0 zeros(1,n);
                   zeros(1,n) 0 0 0 0 1 -1 0 0 0 0;
                   1 0 0 0 0 zeros(1,2*n);
                   zeros(1,2*n) 0 0 0 0 1];
            beq = [zeros(2,1);
                   0;
                   0];
            Aneq = [];
            bneq = [];
            
            n1 = length(time_human1);
            n2 = length(time_human2);
            n3 = length(time_human3);
            nlcon = @(bv) deal([],[bezier2(bv(1:n),1,1)/n1 - bezier2(bv(n+1:2*n),0,1)/n2;
                                   bezier2(bv(n+1:2*n),1,1)/n2 - bezier2(bv(2*n+1:3*n),0,1)/n3]);
            bv_dzcom_all = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            bv_dzcom_phase1 = bv_dzcom_all(1:n);
            bv_dzcom_phase2 = bv_dzcom_all(n+1:2*n);
            bv_dzcom_phase3 = bv_dzcom_all(2*n+1:3*n);
            
            teval = 0:0.01:1;
%             figure; 
%             subplot(2,3,1); hold on; grid on;
%             plot(teval,bezier2(bv_dzcom_phase1,teval))
%             plot(linspace(0,1,length(zcom1)),dzcom1)
%             subplot(2,3,2); hold on; grid on;
%             plot(teval,bezier2(bv_dzcom_phase2,teval))
%             plot(linspace(0,1,length(zcom2)),dzcom2)
%             subplot(2,3,3); hold on; grid on;
%             plot(teval,bezier2(bv_dzcom_phase3,teval))
%             plot(linspace(0,1,length(zcom3)),dzcom3)
%             legend('bezier','raw')
%             
%             subplot(2,3,[4 5 6]); hold on; grid on;
%             plot(time_human1,bezier2(bv_dzcom_phase1,time_norm1))
%             plot(time_human1,dzcom1)
%             plot(time_human2+time_human1(end),bezier2(bv_dzcom_phase2,time_norm2))
%             plot(time_human2+time_human1(end),dzcom2)
%             plot(time_human3+time_human1(end)+time_human2(end),bezier2(bv_dzcom_phase3,time_norm3))
%             plot(time_human3+time_human1(end)+time_human2(end),dzcom3)
%             legend('bezier','raw')
%             linkaxes
            
            allBeziers.(fns{fn}).('phase1').bv_dzcom = bv_dzcom_phase1;
            allBeziers.(fns{fn}).('phase2').bv_dzcom = bv_dzcom_phase2;
            allBeziers.(fns{fn}).('phase3').bv_dzcom = bv_dzcom_phase3;
        end

        bv0 = [ddzcom(1) ddzcom(1) ddzcom(round(end/2)) ddzcom(end) ddzcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,ddzcom);
        bv_ddzcom = fmincon(fun,bv0,[],[],[],[])

        %%%%%%%%%%%%%
        %% XCOM
        if ph == 1
            n = 4;
            xcom1 = xcom;
            xcom2 = allData.(fns{fn}).('phase2').xcom;
            xcom3 = allData.(fns{fn}).('phase3').xcom;
            
            xcom1 = xcom1 - xcom1(1);
            xcom2 = xcom2 - xcom2(1) + xcom1(end);
            xcom3 = xcom3 - xcom3(1) + xcom2(end);
            
            time_norm1 = time_norm;
            time_norm2 = allData.(fns{fn}).('phase2').time_norm;
            time_norm3 = allData.(fns{fn}).('phase3').time_norm;
            time_human1 = time;
            time_human2 = allData.(fns{fn}).('phase2').time;
            time_human3 = allData.(fns{fn}).('phase3').time;
            
            bv0 = [xcom1(1) xcom1(1) xcom1(end) xcom1(end) ...
                   xcom2(1) xcom2(1) xcom2(end) xcom2(end) ...
                   xcom3(1) xcom3(1) xcom3(end) xcom3(end)];
            fun = @(x) costFcnBezierZ(x,time_norm1,time_norm2,time_norm3,...
                                        xcom1,xcom2,xcom3,...
                                        n,n,n);
            Aeq = [0 0 0 1 -1 0 0 0 zeros(1,n);
                   zeros(1,n) 0 0 0 1 -1 0 0 0;
                   1 0 0 0 zeros(1,2*n);
                   zeros(1,2*n) 0 0 0 1];
            beq = [zeros(2,1);
                   0;
                   xcom3(end)];
            Aneq = [];
            bneq = [];
            
            n1 = length(time_human1);
            n2 = length(time_human2);
            n3 = length(time_human3);
            nlcon = @(bv) deal([],[bezier2(bv(1:n),1,1)/n1 - bezier2(bv(n+1:2*n),0,1)/n2;
                                   bezier2(bv(n+1:2*n),1,1)/n2 - bezier2(bv(2*n+1:3*n),0,1)/n3]);
            bv_xcom_all = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            bv_xcom_phase1 = bv_xcom_all(1:n);
            bv_xcom_phase2 = bv_xcom_all(n+1:2*n);
            bv_xcom_phase3 = bv_xcom_all(2*n+1:3*n);
            
            teval = 0:0.01:1;
%             figure; 
%             subplot(2,3,1); hold on; grid on;
%             plot(teval,bezier2(bv_xcom_phase1,teval))
%             plot(linspace(0,1,length(xcom1)),xcom1)
%             subplot(2,3,2); hold on; grid on;
%             plot(teval,bezier2(bv_xcom_phase2,teval))
%             plot(linspace(0,1,length(xcom2)),xcom2)
%             subplot(2,3,3); hold on; grid on;
%             plot(teval,bezier2(bv_xcom_phase3,teval))
%             plot(linspace(0,1,length(xcom3)),xcom3)
%             legend('bezier','raw')
%             
%             subplot(2,3,[4 5 6]); hold on; grid on;
%             plot(time_human1,bezier2(bv_xcom_phase1,time_norm1))
%             plot(time_human1,xcom1)
%             plot(time_human2+time_human1(end),bezier2(bv_xcom_phase2,time_norm2))
%             plot(time_human2+time_human1(end),xcom2)
%             plot(time_human3+time_human1(end)+time_human2(end),bezier2(bv_xcom_phase3,time_norm3))
%             plot(time_human3+time_human1(end)+time_human2(end),xcom3)
%             legend('bezier','raw')
%             linkaxes
            
            allBeziers.(fns{fn}).('phase1').bv_xcom = bv_xcom_phase1;
            allBeziers.(fns{fn}).('phase2').bv_xcom = bv_xcom_phase2;
            allBeziers.(fns{fn}).('phase3').bv_xcom = bv_xcom_phase3;
        end

        bv0 = [dxcom(1) dxcom(1) dxcom(round(end/2)) dxcom(end) dxcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,dxcom);
        bv_dxcom = fmincon(fun,bv0,[],[],[],[])
        
        bv0 = [ddxcom(1) ddxcom(1) ddxcom(round(end/2)) ddxcom(end) ddxcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,ddxcom);
        bv_ddxcom = fmincon(fun,bv0,[],[],[],[])
        
        % to struct
%         allBeziersHuman.(fns{fn}).(phs{ph}).bv_zcom = bv_zcom;
%         allBeziers.(fns{fn}).(phs{ph}).bv_dzcom = bv_dzcom;
        allBeziers.(fns{fn}).(phs{ph}).bv_ddzcom = bv_ddzcom;
%         allBeziers.(fns{fn}).(phs{ph}).bv_xcom = bv_xcom;
        allBeziers.(fns{fn}).(phs{ph}).bv_dxcom = bv_dxcom;
        allBeziers.(fns{fn}).(phs{ph}).bv_ddxcom = bv_ddxcom;
        allBeziers.(fns{fn}).(phs{ph}).timeMax = time(end);
        allBeziers.(fns{fn}).(phs{ph}).timeMax_SSP = time_SSP(end);
    end
end
end


function [allDataHuman,allDataCassie] = orderHumanData(exp,stepHeight,allDataHuman,allDataCassie)
filename = strcat('data/human/humanDataCoM.mat');
load(filename)

filename = strcat('data/human/timeEvents.mat');
load(filename)

downstep = stepHeight;

%% Here we are only ordering data so unexp00 = exp00 (and we don't actually have unexp00 trial)
% expInd = exp;
% if isequal(exp,'unexp00')
%     exp = 'exp00';
% end

%% phasing indices
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
