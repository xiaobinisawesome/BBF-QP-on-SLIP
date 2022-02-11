clearvars; close all; clc;

%% Generate downstep data
addpath('functions')

exps = {'exp25','exp50','exp75','exp100',...
        'unexp25','unexp50','unexp75','unexp100'};
stepHeights = [0.025 0.050 0.075 0.100 0.025 0.050 0.075 0.100];


downstepDataHuman = struct;
downstepDataCassie = struct;

% figure(10101); hold on; grid on;
for i = 1:length(exps)
    [downstepDataHuman,downstepDataCassie] = orderHumanData(exps{i},stepHeights(i),downstepDataHuman,downstepDataCassie);
end

% add exp00 data
load('data/beziers/nominalDataHumanAsDownstep.mat')
downstepDataHuman.exp00 = nominalDataHumanAsDownstep.exp00;
% reorder so exp00 at start
order = [2 3 4 5 6 7 8 9 1];
exps = {'exp00','exp25','exp50','exp75','exp100',...
           'unexp25','unexp50','unexp75','unexp100'};
stepHeights = [0.0 0.025 0.050 0.075 0.100 0.025 0.050 0.075 0.100];
for i = 1:length(exps)
    allDataHuman.(exps{i}) = downstepDataHuman.(exps{i});
end

%%% TODO: add exp00 for Cassie
allDataCassie = downstepDataCassie;

%% Bezier curves
allData = allDataHuman;
save('data/beziers/allDataHuman.mat','allData')
allData = allDataCassie;
save('data/beziers/allDataCassie.mat','allData')

allBeziersHuman = struct;
allBeziersCassie = struct;

    allBeziersHuman = fitDataToBezier(allDataHuman,allBeziersHuman);
    allBeziersCassie = fitDataToBezier(allDataCassie,allBeziersCassie);

allBeziers = allBeziersHuman;
save('data/beziers/allBeziersHuman.mat','allBeziers')
allBeziers = allBeziersCassie;
save('data/beziers/allBeziersCassie.mat','allBeziers')


%% FUNCTIONS
function allBeziers = fitDataToBezier(allData,allBeziers)
persistent Fsave
if isempty(Fsave)
    Fsave = 0;
end


fns = fieldnames(allData);
for i = 1:length(fns) % exp25, exp50, ...
    fns2 = fieldnames(allData.(fns{i}));
    for ii = 1:length(fns2) % phase1, phase2, phase3
        time = allData.(fns{i}).(fns2{ii}).time;
        time_SSP = allData.(fns{i}).(fns2{ii}).time_SSP;
        time_DSP = allData.(fns{i}).(fns2{ii}).time_DSP;
        time_norm = allData.(fns{i}).(fns2{ii}).time_norm;
        time_norm_SSP = allData.(fns{i}).(fns2{ii}).time_norm_SSP;
        time_norm_DSP = allData.(fns{i}).(fns2{ii}).time_norm_DSP;
        
        zcom = allData.(fns{i}).(fns2{ii}).zcom;
        dzcom = allData.(fns{i}).(fns2{ii}).dzcom;
        ddzcom = allData.(fns{i}).(fns2{ii}).ddzcom;
        
        xcom = allData.(fns{i}).(fns2{ii}).xcom;
        dxcom = allData.(fns{i}).(fns2{ii}).dxcom;
        ddxcom = allData.(fns{i}).(fns2{ii}).ddxcom;
        
        grf_st = allData.(fns{i}).(fns2{ii}).grf_st;
        grf_sw = allData.(fns{i}).(fns2{ii}).grf_sw;
        grf_SSP = allData.(fns{i}).(fns2{ii}).grf_SSP;
        grf_DSP_st = allData.(fns{i}).(fns2{ii}).grf_DSP_st;
        grf_DSP_sw = allData.(fns{i}).(fns2{ii}).grf_DSP_sw;
        
        dgrf_sw = [0 diff(grf_st)./diff(time)];
        dgrf_st = [diff(grf_sw)./diff(time) 0];
        dgrf_SSP = [diff(grf_SSP)./diff(time_SSP) 0];
        dgrf_DSP_st = [0 diff(grf_DSP_st)./diff(time_DSP)];
        dgrf_DSP_sw = [diff(grf_DSP_sw)./diff(time_DSP) 0];

        %%%%%%%%%%%%%
        % ZCOM
        mid = round(length(zcom)/2);
        
        bv0 = [zcom(1) zcom(1) zcom(1) zcom(end) zcom(end)];
        fun = @(x) costFcnBezier(x,time_norm,zcom);
        bv_zcom = fmincon(fun,bv0,[],[],[],[])

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

        %%%%%%%%%%%%%
        % GRF
        bv0 = [grf_st(1) grf_st(1) grf_st(mid) grf_st(end) grf_st(end)];
        fun = @(x) costFcnBezier(x,time_norm,grf_st);
        bv_grf_st = fmincon(fun,bv0,[],[],[],[])

        bv0 = [grf_sw(1) grf_sw(1) grf_sw(mid) grf_sw(end) grf_sw(end)];
        fun = @(x) costFcnBezier(x,time_norm,grf_sw);
        bv_grf_sw = fmincon(fun,bv0,[],[],[],[])

%         bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(10) grf_SSP(end) grf_SSP(end)];
%         fun = @(x) costFcnBezier(x,time_norm_SSP,grf_SSP);
%         bv_grf_SSP = fmincon(fun,bv0,[],[],SSPAeq,SSPbeq)
% 
%         bv0 = [grf_DSP_st(1) grf_DSP_st(1) grf_DSP_st(mid) grf_DSP_st(end) grf_DSP_st(end)];
%         fun = @(x) costFcnBezier(x,time_norm_SSP,grf_DSP_st);
%         bv_grf_DSP_st = fmincon(fun,bv0,[],[],DSPstAeq,DSPstbeq)
% 
%         bv0 = [grf_DSP_sw(1) grf_DSP_sw(1) grf_DSP_sw(mid) grf_DSP_sw(end) grf_DSP_sw(end)];
%         fun = @(x) costFcnBezier(x,time_norm_SSP,grf_DSP_sw);
%         bv_grf_DSP_sw = fmincon(fun,bv0,[],[],DSPswAeq,DSPswbeq)

        FSSP1 = 350;
        FSSP2 = 350;
        FSSP3 = 426;
        if ii == 1
%             bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(10) grf_SSP(end) grf_SSP(end) ...
%                    grf_DSP_st(1) grf_DSP_st(1) grf_DSP_st(mid) grf_DSP_st(end) grf_DSP_st(end)...
%                    grf_DSP_sw(1) grf_DSP_sw(1) grf_DSP_sw(mid) grf_DSP_sw(end) grf_DSP_sw(end)];
%             fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
%                                         grf_SSP,grf_DSP_st,grf_DSP_sw);
%             Aeq = [1 -1 0 0 0 zeros(1,10);
%                    0 0 0 1 -1 zeros(1,10);
%                    zeros(1,5) 1 -1 0 0 0 zeros(1,5);
%                    zeros(1,10) 0 0 0 1 -1;
%                    0 0 0 0 1 -1 0 0 0 0 zeros(1,5); % SSPf = DSP0
%                    1 0 0 0 0 zeros(1,10)];
%             beq = [zeros(5,1);
%                    FSSP1];
%             Aneq = [0 0 1 -1 0 zeros(1,10)];
%             bneq = [0];
            bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(end) grf_SSP(end) ...
                   grf_DSP_st(1) grf_DSP_st(1) grf_DSP_st(end) grf_DSP_st(end)...
                   grf_DSP_sw(1) grf_DSP_sw(1) grf_DSP_sw(end) grf_DSP_sw(end)];
            fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
                                        grf_SSP,grf_DSP_st,grf_DSP_sw,...
                                        4,4,4);
            Aeq = [1 -1 0 0 zeros(1,8);
                   0 0  1 -1 zeros(1,8);
                   zeros(1,4) 1 -1 0 0 zeros(1,4);
                   zeros(1,8) 0 0 1 -1;
                   0 0 0 1 -1 0 0 0 zeros(1,4); % SSPf = DSP0
                   1 0 0 0 zeros(1,8)];
            beq = [zeros(5,1);
                   FSSP1];
            Aneq = [];
            bneq = [];
            
            bv_grf_phase1 = fmincon(fun,bv0,Aneq,bneq,Aeq,beq)
            
            bv_grf_SSP = bv_grf_phase1(1:4);
            bv_grf_DSP_st = bv_grf_phase1(5:8);
            bv_grf_DSP_sw = bv_grf_phase1(9:12);
        elseif ii == 2
%             bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(10) grf_SSP(end) grf_SSP(end) ...
%                    grf_DSP_st(1) grf_DSP_st(1) grf_DSP_st(mid) grf_DSP_st(end) grf_DSP_st(end)...
%                    grf_DSP_sw(1) grf_DSP_sw(1) grf_DSP_sw(mid) grf_DSP_sw(end) grf_DSP_sw(end)];
%             fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
%                                         grf_SSP,grf_DSP_st,grf_DSP_sw);
%             Aeq = [1 -1 0 0 0 zeros(1,10);
%                    0 0 0 1 -1 zeros(1,10);
%                    zeros(1,5) 1 -1 0 0 0 zeros(1,5);
%                    zeros(1,10) 0 0 0 1 -1;
%                    0 0 0 0 1 -1 0 0 0 0 zeros(1,5); % SSPf = DSP0
%                    1 0 0 0 0 zeros(1,10)];
%             beq = [zeros(5,1);
%                    Fsave];
%             Aneq = [];
%             bneq = [];
            bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(10) grf_SSP(end) grf_SSP(end) ...
                   grf_DSP_st(1) grf_DSP_st(1) grf_DSP_st(end) grf_DSP_st(end)...
                   grf_DSP_sw(1) grf_DSP_sw(1) grf_DSP_sw(end) grf_DSP_sw(end)];
            fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
                                        grf_SSP,grf_DSP_st,grf_DSP_sw,...
                                        5,4,4);
            Aeq = [1 -1 0 0 0 zeros(1,8);
                   0 0 0 1 -1 zeros(1,8);
                   zeros(1,5) 1 -1 0 0 zeros(1,4);
                   zeros(1,9) 0 0 1 -1;
                   0 0 0 0 1 -1 0 0 0 zeros(1,4); % SSPf = DSP0
                   1 0 0 0 0 zeros(1,8)];
            beq = [zeros(5,1);
                   Fsave];
            Aneq = [];
            bneq = [];
            
            nlcon = @(bv) deal([],[bezier2(bv(1:5),0.5) - FSSP2]);
            bv_grf_phase1 = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon)
            
            bv_grf_SSP = bv_grf_phase1(1:5);
            bv_grf_DSP_st = bv_grf_phase1(6:9);
            bv_grf_DSP_sw = bv_grf_phase1(10:13);
        else
%             bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(10) grf_SSP(end) grf_SSP(end) ...
%                    0 0 0 0 0 ...
%                    0 0 0 0 0];
%             fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
%                                         grf_SSP,grf_DSP_st,grf_DSP_sw);
%             Aeq = [1 -1 0 0 0 zeros(1,10);
%                    0 0 0 1 -1 zeros(1,10);
%                    1 0 0 0 0 zeros(1,10);
%                    0 0 0 0 1 zeros(1,10)];
%             beq = [zeros(2,1)
%                    Fsave;
%                    FSSP3];
%             Aneq = [];
%             bneq = [];
            bv0 = [grf_SSP(1) grf_SSP(1) grf_SSP(end) grf_SSP(end) ...
                   0 0 0 0 ...
                   0 0 0 0];
            fun = @(x) costFcnBezierF(x,time_norm_SSP,time_norm_DSP,time_norm_DSP,...
                                        grf_SSP,grf_DSP_st,grf_DSP_sw,...
                                        4,4,4);
            Aeq = [1 -1 0 0 zeros(1,8);
                   0 0 1 -1 zeros(1,8);
                   1 0 0 0 zeros(1,8);
                   0 0 0 1 zeros(1,8)];
            beq = [zeros(2,1)
                   Fsave;
                   FSSP3];
            Aneq = [];
            bneq = [];
            
            bv_grf_phase1 = fmincon(fun,bv0,Aneq,bneq,Aeq,beq)
            
            bv_grf_SSP = bv_grf_phase1(1:4);
            bv_grf_DSP_st = bv_grf_phase1(5:8);
            bv_grf_DSP_sw = bv_grf_phase1(9:12);
        end
        
        
        if ii == 1
            Fsave = bv_grf_DSP_sw(end);
        elseif ii == 2
            Fsave = bv_grf_DSP_sw(end);
        end
        
        n = 0:0.01:1;
        figure
        subplot(1,3,1)
        plot(n,bezier2(bv_grf_SSP,n))
        subplot(1,3,2)
        plot(n,bezier2(bv_grf_DSP_st,n))
        subplot(1,3,3)
        plot(n,bezier2(bv_grf_DSP_sw,n))

        
        %%%%%%%%%%%%
        % dGRF
        bv0 = [dgrf_st(1) dgrf_st(1) dgrf_st(mid) dgrf_st(end) dgrf_st(end)];
        fun = @(x) costFcnBezier(x,time_norm,dgrf_st);
        bv_dgrf_st = fmincon(fun,bv0,[],[],[],[])

        bv0 = [dgrf_sw(1) dgrf_sw(1) dgrf_sw(mid) dgrf_sw(end) dgrf_sw(end)];
        fun = @(x) costFcnBezier(x,time_norm,dgrf_sw);
        bv_dgrf_sw = fmincon(fun,bv0,[],[],[],[])

        mid = round(length(dgrf_SSP)/2);
        bv0 = [dgrf_SSP(1) dgrf_SSP(1) dgrf_SSP(mid) dgrf_SSP(end) dgrf_SSP(end)];
        fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_SSP);
        bv_dgrf_SSP = fmincon(fun,bv0,[],[],[],[])

        if ii == 3
            bv_dgrf_DSP_st = zeros(1,5)
            bv_dgrf_DSP_sw = zeros(1,5)
        else
            bv0 = [dgrf_DSP_st(1) dgrf_DSP_st(1) dgrf_DSP_st(mid) dgrf_DSP_st(end) dgrf_DSP_st(end)];
            fun = @(x) costFcnBezier(x,time_norm_DSP,dgrf_DSP_st);
            bv_dgrf_DSP_st = fmincon(fun,bv0,[],[],[],[])

            bv0 = [dgrf_DSP_sw(1) dgrf_DSP_sw(1) dgrf_DSP_sw(mid) dgrf_DSP_sw(end) dgrf_DSP_sw(end)];
            fun = @(x) costFcnBezier(x,time_norm_DSP,dgrf_DSP_sw);
            bv_dgrf_DSP_sw = fmincon(fun,bv0,[],[],[],[])
        end
        
        % to struct
        allBeziers.(fns{i}).(fns2{ii}).bv_zcom = bv_zcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_dzcom = bv_dzcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_ddzcom = bv_ddzcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_xcom = bv_xcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_dxcom = bv_dxcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_ddxcom = bv_ddxcom;
        allBeziers.(fns{i}).(fns2{ii}).bv_grf_st = bv_grf_st;
        allBeziers.(fns{i}).(fns2{ii}).bv_grf_sw = bv_grf_sw;
        allBeziers.(fns{i}).(fns2{ii}).bv_grf_SSP = bv_grf_SSP;
        allBeziers.(fns{i}).(fns2{ii}).bv_grf_DSP_st = bv_grf_DSP_st;
        allBeziers.(fns{i}).(fns2{ii}).bv_grf_DSP_sw = bv_grf_DSP_sw;
        allBeziers.(fns{i}).(fns2{ii}).bv_dgrf_st = bv_dgrf_st;
        allBeziers.(fns{i}).(fns2{ii}).bv_dgrf_sw = bv_dgrf_sw;
        allBeziers.(fns{i}).(fns2{ii}).bv_dgrf_SSP = bv_dgrf_SSP;
        allBeziers.(fns{i}).(fns2{ii}).bv_dgrf_DSP_st = bv_dgrf_DSP_st;
        allBeziers.(fns{i}).(fns2{ii}).bv_dgrf_DSP_sw = bv_dgrf_DSP_sw;
        allBeziers.(fns{i}).(fns2{ii}).timeMax = time(end);
        allBeziers.(fns{i}).(fns2{ii}).timeMax_SSP = time_SSP(end);
        allBeziers.(fns{i}).(fns2{ii}).timeMax_DSP = time_DSP(end);
    end
end
end


function [downstepDataHuman,downstepDataCassie] = orderHumanData(exp,stepHeight,downstepDataHuman,downstepDataCassie)
filename = strcat('data/collocation/aSLIP_',exp,'_1.mat');
load(filename)

downstep = stepHeight;

%% timing
time_human_1 = [SSP1sol.t(1:end) ...
                SSP1sol.t(end)+DSP1sol.t(2:end)];
time_human_SSP_1 = SSP1sol.t;
time_human_DSP_1 = SSP1sol.t(end)+DSP1sol.t(1:end);
time_norm_1 = time_human_1./(time_human_1(end));
time_norm_SSP_1 = (SSP1sol.t - SSP1sol.t(1))/(SSP1sol.t(end) - SSP1sol.t(1));
time_norm_DSP_1 = (time_human_DSP_1-time_human_DSP_1(1))/(time_human_DSP_1(end)-time_human_DSP_1(1));

time_human_2 = [SSP2sol.t(1:end)-SSP2sol.t(1) ...
                DSP2sol.t(2:end)+SSP2sol.t(end)-SSP2sol.t(1)];
time_human_SSP_2 = SSP2sol.t-SSP2sol.t(1);
time_human_DSP_2 = DSP2sol.t(1:end)+SSP2sol.t(end)-SSP2sol.t(1);
time_norm_2 = time_human_2./(time_human_2(end));
time_norm_SSP_2 = (SSP2sol.t - SSP2sol.t(1))/(SSP2sol.t(end) - SSP2sol.t(1));
time_norm_DSP_2 = (time_human_DSP_2-time_human_DSP_2(1))/(time_human_DSP_2(end)-time_human_DSP_2(1));

time_human_3 = SSP3sol.t(1:end);
time_human_SSP_3 = SSP3sol.t;
time_human_DSP_3 = [0];
time_norm_3 = time_human_3./(time_human_3(end));
time_norm_SSP_3 = [(SSP3sol.t - SSP3sol.t(1))/(SSP3sol.t(end) - SSP3sol.t(1))];
time_norm_DSP_3 = [0];

%% Outputs
% step 1
xcom_human_1 = [SSP1sol.x DSP1sol.x(2:end)];
dxcom_human_1 = [SSP1sol.dx DSP1sol.dx(2:end)];
ddxcom_human_1 = [SSP1sol.ddx DSP1sol.ddx(2:end)];

zcom_human_1 = [SSP1sol.z DSP1sol.z(2:end)];
dzcom_human_1 = [SSP1sol.dz DSP1sol.dz(2:end)];
ddzcom_human_1 = [SSP1sol.ddz DSP1sol.ddz(2:end)];

grf_human_st_1 = [SSP1sol.GRF(1,:) DSP1sol.leg1.GRF(1,2:end)];
grf_human_sw_1 = [SSP1sol.GRF(2,:) DSP1sol.leg1.GRF(2,2:end)];

grf_human_SSP_1 = SSP1sol.GRF(1,:);
grf_human_DSP_st_1 = DSP1sol.leg1.GRF(1,:);
grf_human_DSP_sw_1 = DSP1sol.leg1.GRF(2,:);

% step 2
xcom_human_2 = [SSP2sol.x(1:end) DSP2sol.x(2:end)];
dxcom_human_2 = [SSP2sol.dx(1:end) DSP2sol.dx(2:end)];
ddxcom_human_2 = [SSP2sol.ddx(1:end) DSP2sol.ddx(2:end)];

zcom_human_2 = [SSP2sol.z(1:end)-downstep DSP2sol.z(2:end)-downstep];
dzcom_human_2 = [SSP2sol.x(1:end) DSP2sol.dz(2:end)];
ddzcom_human_2 = [SSP2sol.x(1:end) DSP2sol.ddz(2:end)];

grf_human_st_2 = [SSP2sol.GRF(2,1:end) DSP2sol.leg1.GRF(2,2:end)];
grf_human_sw_2 = [SSP2sol.GRF(1,1:end) DSP2sol.leg1.GRF(1,2:end)];

grf_human_SSP_2 = SSP2sol.GRF(2,:);
grf_human_DSP_st_2 = DSP2sol.leg1.GRF(2,:);
grf_human_DSP_sw_2 = DSP2sol.leg1.GRF(1,:);

% step 3
xcom_human_3 = [SSP3sol.x(1:end)];
dxcom_human_3 = [SSP3sol.dx(1:end)];
ddxcom_human_3 = [SSP3sol.ddx(1:end)];

zcom_human_3 = [SSP3sol.z(1:end)];
dzcom_human_3 = [SSP3sol.dz(1:end)];
ddzcom_human_3 = [SSP3sol.ddz(1:end)];

grf_human_st_3 = [SSP3sol.GRF(1,1:end)];
grf_human_sw_3 = [SSP3sol.GRF(2,1:end)];

grf_human_SSP_3 = SSP3sol.GRF(1,:);
grf_human_DSP_st_3 = [0];
grf_human_DSP_sw_3 = [0];

% figure(10101);
% if any(strcmp(exp,{'exp00','unexp25','unexp50','unexp75','unexp100'}))
%     plot(time_human_1,zcom_human_1)
% end

%% Save all data as struct
downstepDataHuman.(exp).phase1.time = time_human_1;
downstepDataHuman.(exp).phase1.time_SSP = time_human_SSP_1;
downstepDataHuman.(exp).phase1.time_DSP = time_human_DSP_1;
downstepDataHuman.(exp).phase1.time_norm = time_norm_1;
downstepDataHuman.(exp).phase1.time_norm_SSP = time_norm_SSP_1;
downstepDataHuman.(exp).phase1.time_norm_DSP = time_norm_DSP_1;
downstepDataHuman.(exp).phase1.xcom = xcom_human_1;
downstepDataHuman.(exp).phase1.dxcom = dxcom_human_1;
downstepDataHuman.(exp).phase1.ddxcom = ddxcom_human_1;
downstepDataHuman.(exp).phase1.zcom = zcom_human_1;
downstepDataHuman.(exp).phase1.dzcom = dzcom_human_1;
downstepDataHuman.(exp).phase1.ddzcom = ddzcom_human_1;
downstepDataHuman.(exp).phase1.grf_st = grf_human_st_1;
downstepDataHuman.(exp).phase1.grf_sw = grf_human_sw_1;
downstepDataHuman.(exp).phase1.grf_SSP = grf_human_SSP_1;
downstepDataHuman.(exp).phase1.grf_DSP_st = grf_human_DSP_st_1;
downstepDataHuman.(exp).phase1.grf_DSP_sw = grf_human_DSP_sw_1;

downstepDataHuman.(exp).phase2.time = time_human_2;
downstepDataHuman.(exp).phase2.time_SSP = time_human_SSP_2;
downstepDataHuman.(exp).phase2.time_DSP = time_human_DSP_2;
downstepDataHuman.(exp).phase2.time_norm = time_norm_2;
downstepDataHuman.(exp).phase2.time_norm_SSP = time_norm_SSP_2;
downstepDataHuman.(exp).phase2.time_norm_DSP = time_norm_DSP_2;
downstepDataHuman.(exp).phase2.xcom = xcom_human_2;
downstepDataHuman.(exp).phase2.dxcom = dxcom_human_2;
downstepDataHuman.(exp).phase2.ddxcom = ddxcom_human_2;
downstepDataHuman.(exp).phase2.zcom = zcom_human_2;
downstepDataHuman.(exp).phase2.dzcom = dzcom_human_2;
downstepDataHuman.(exp).phase2.ddzcom = ddzcom_human_2;
downstepDataHuman.(exp).phase2.grf_st = grf_human_st_2;
downstepDataHuman.(exp).phase2.grf_sw = grf_human_sw_2;
downstepDataHuman.(exp).phase2.grf_SSP = grf_human_SSP_2;
downstepDataHuman.(exp).phase2.grf_DSP_st = grf_human_DSP_st_2;
downstepDataHuman.(exp).phase2.grf_DSP_sw = grf_human_DSP_sw_2;

downstepDataHuman.(exp).phase3.time = time_human_3;
downstepDataHuman.(exp).phase3.time_SSP = time_human_SSP_3;
downstepDataHuman.(exp).phase3.time_DSP = time_human_DSP_3;
downstepDataHuman.(exp).phase3.time_norm = time_norm_3;
downstepDataHuman.(exp).phase3.time_norm_SSP = time_norm_SSP_3;
downstepDataHuman.(exp).phase3.time_norm_DSP = time_norm_DSP_3;
downstepDataHuman.(exp).phase3.xcom = xcom_human_3;
downstepDataHuman.(exp).phase3.dxcom = dxcom_human_3;
downstepDataHuman.(exp).phase3.ddxcom = ddxcom_human_3;
downstepDataHuman.(exp).phase3.zcom = zcom_human_3;
downstepDataHuman.(exp).phase3.dzcom = dzcom_human_3;
downstepDataHuman.(exp).phase3.ddzcom = ddzcom_human_3;
downstepDataHuman.(exp).phase3.grf_st = grf_human_st_3;
downstepDataHuman.(exp).phase3.grf_sw = grf_human_sw_3;
downstepDataHuman.(exp).phase3.grf_SSP = grf_human_SSP_3;
downstepDataHuman.(exp).phase3.grf_DSP_st = grf_human_DSP_st_3;
downstepDataHuman.(exp).phase3.grf_DSP_sw = grf_human_DSP_sw_3;

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
    time_cassie_DSP = Ts_scaling*eval(strcat('time_human_DSP_',add));
    time_norm = eval(strcat('time_norm_',add));
    time_norm_SSP = eval(strcat('time_norm_SSP_',add));
    time_norm_DSP = eval(strcat('time_norm_DSP_',add));
    
    xcom_cassie = x_scaling*eval(strcat('xcom_human_',add));
    dxcom_cassie = dx_scaling*eval(strcat('dxcom_human_',add));
    ddxcom_cassie = ddx_scaling*eval(strcat('ddxcom_human_',add));
    
    L_human = sqrt(eval(strcat('xcom_human_',add)).^2 + ...
                   eval(strcat('zcom_human_',add)).^2);
    L_cassie = L_scaling*L_human;
    zcom_cassie = sqrt(L_cassie.^2 - xcom_cassie.^2);
    dzcom_cassie = dz_scaling*eval(strcat('dzcom_human_',add));
    ddzcom_cassie = ddz_scaling*eval(strcat('ddzcom_human_',add));
    
    grf_cassie_st = grf_scaling*eval(strcat('grf_human_st_',add));
    grf_cassie_sw = grf_scaling*eval(strcat('grf_human_sw_',add));
    grf_cassie_SSP = grf_scaling*eval(strcat('grf_human_SSP_',add));
    grf_cassie_DSP_st = grf_scaling*eval(strcat('grf_human_DSP_st_',add));
    grf_cassie_DSP_sw = grf_scaling*eval(strcat('grf_human_DSP_sw_',add));
    
    downstepDataCassie.(exp).(strcat('phase',add)).time = time_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).time_SSP = time_cassie_SSP;
    downstepDataCassie.(exp).(strcat('phase',add)).time_DSP = time_cassie_DSP;
    downstepDataCassie.(exp).(strcat('phase',add)).time_norm = time_norm;
    downstepDataCassie.(exp).(strcat('phase',add)).time_norm_SSP = time_norm_SSP;
    downstepDataCassie.(exp).(strcat('phase',add)).time_norm_DSP = time_norm_DSP;
    downstepDataCassie.(exp).(strcat('phase',add)).xcom = xcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).dxcom = dxcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).ddxcom = ddxcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).zcom = zcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).dzcom = dzcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).ddzcom = ddzcom_cassie;
    downstepDataCassie.(exp).(strcat('phase',add)).grf_st = grf_cassie_st;
    downstepDataCassie.(exp).(strcat('phase',add)).grf_sw = grf_cassie_sw;
    downstepDataCassie.(exp).(strcat('phase',add)).grf_SSP = grf_cassie_SSP;
    downstepDataCassie.(exp).(strcat('phase',add)).grf_DSP_st = grf_cassie_DSP_st;
    downstepDataCassie.(exp).(strcat('phase',add)).grf_DSP_sw = grf_cassie_DSP_sw;
    
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
