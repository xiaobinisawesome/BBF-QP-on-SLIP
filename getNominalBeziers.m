% clear all; close all; clc;
addpath('functions/')

%% Create Human data
load('data/collocation/aSLIP_nominal_human.mat')
time_human = [SSP1sol.t SSP1sol.t(end)+DSP1sol.t(2:end)];
time_norm = time_human./(time_human(end));

time_human_SSP = SSP1sol.t;
time_human_DSP = SSP1sol.t(end)+DSP1sol.t(1:end);
time_norm_SSP = time_human_SSP./(time_human_SSP(end));
time_norm_DSP = (time_human_DSP-time_human_DSP(1))/(time_human_DSP(end)-time_human_DSP(1));

th_human = [atan2(SSP1sol.z,SSP1sol.x) atan2(DSP1sol.z(2:end),DSP1sol.x(2:end))];

xcom_human = [SSP1sol.x DSP1sol.x(2:end)];
dxcom_human = [SSP1sol.dx DSP1sol.dx(2:end)];
ddxcom_human = [SSP1sol.ddx DSP1sol.ddx(2:end)];

zcom_human = [SSP1sol.z DSP1sol.z(2:end)] - 0.01;
dzcom_human = [SSP1sol.dz DSP1sol.dz(2:end)];
ddzcom_human = [SSP1sol.ddz DSP1sol.ddz(2:end)];

grf_human_st = [SSP1sol.GRF(1,:) DSP1sol.leg1.GRF(1,2:end)];
grf_human_sw = [SSP1sol.GRF(2,:) DSP1sol.leg1.GRF(2,2:end)];
grf_human_SSP = SSP1sol.GRF(1,:);
grf_human_DSP_st = DSP1sol.leg1.GRF(1,:);
grf_human_DSP_sw = DSP1sol.leg1.GRF(2,:);

dgrf_human_sw = [0 diff(grf_human_st)./diff(time_human)];
dgrf_human_st = [diff(grf_human_sw)./diff(time_human) 0];
dgrf_human_SSP = [diff(grf_human_SSP)./diff(time_human_SSP) 0];
dgrf_human_DSP_st = [0 diff(grf_human_DSP_st)./diff(time_human_SSP)];
dgrf_human_DSP_sw = [diff(grf_human_DSP_sw)./diff(time_human_SSP) 0];

L_human = sqrt(zcom_human.^2 + xcom_human.^2);

%%%%%%%%%%%%%%%%%%%%%
%%% HUMAN SPLINES %%%
%%%%%%%%%%%%%%%%%%%%%

%% Save data to .mat file
nominalDataHuman.exp00.time = time_human;
nominalDataHuman.exp00.time_SSP = time_human_SSP;
nominalDataHuman.exp00.time_DSP = time_human_DSP;
nominalDataHuman.exp00.time_norm = time_norm;
nominalDataHuman.exp00.time_norm_SSP = time_norm_SSP;
nominalDataHuman.exp00.time_norm_DSP = time_norm_DSP;
nominalDataHuman.exp00.xcom = xcom_human;
nominalDataHuman.exp00.dxcom = dxcom_human;
nominalDataHuman.exp00.ddxcom = ddxcom_human;
nominalDataHuman.exp00.zcom = zcom_human;
nominalDataHuman.exp00.dzcom = dzcom_human;
nominalDataHuman.exp00.ddzcom = ddzcom_human;
nominalDataHuman.exp00.grf_st = grf_human_st;
nominalDataHuman.exp00.grf_sw = grf_human_sw;
nominalDataHuman.exp00.grf_SSP = grf_human_SSP;
nominalDataHuman.exp00.grf_DSP_st = grf_human_DSP_st;
nominalDataHuman.exp00.grf_DSP_sw = grf_human_DSP_sw;
save('data/beziers/nominalDataHuman.mat','nominalDataHuman')

%% Create Human splines
figure

%%%%%%%%%%%%%
% ZCOM
teval = linspace(0,1,100);

difftime = diff(time_norm);
bv0 = [zcom_human(1) zcom_human(1) zcom_human(20) zcom_human(end) zcom_human(end)];
fun = @(x) costFcnBezier(x,time_norm,zcom_human);
nlcon = @(bv) deal([],[bezier2(bv,0,1) - bezier2(bv,1,1)]);
bv_zcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0],[],[],nlcon)
subplot(4,3,1); hold on; grid on;
plot(time_norm,zcom_human)
plot(teval,evalBezier(bv_zcom,teval))
legend('data','bezier')
title('zcom')

% figure; hold on; grid on;
% plot(teval,evalBezier(bv_zcom,teval))
% plot(teval+teval(end),evalBezier(bv_zcom,teval))

bv0 = [dzcom_human(1) dzcom_human(1) dzcom_human(20) dzcom_human(end) dzcom_human(end)];
fun = @(x) costFcnBezier(x,time_norm,dzcom_human);
bv_dzcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,2); hold on; grid on;
plot(time_norm,dzcom_human)
plot(time_norm,evalBezier(bv_dzcom,time_norm))
title('dzcom')


ddzcom_human_tmp = ddzcom_human(1:25);
time_norm_ddzcom = time_norm(1:25)/time_norm(25);
% bv0 = [ddzcom_human(1) ddzcom_human(1) ddzcom_human(20) ddzcom_human(end) ddzcom_human(end)];
% fun = @(x) costFcnBezier(x,time_norm,ddzcom_human);
% bv_ddzcom = fmincon(fun,bv0,[],[],[],[])
bv0 = [ddzcom_human_tmp(1) ddzcom_human_tmp(1) ddzcom_human_tmp(10) ddzcom_human_tmp(end) ddzcom_human_tmp(end)];
fun = @(x) costFcnBezier(x,time_norm_ddzcom,ddzcom_human_tmp);
bv_ddzcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,3); hold on; grid on;
plot(time_norm_ddzcom,ddzcom_human_tmp)
plot(time_norm_ddzcom,evalBezier(bv_ddzcom,time_norm_ddzcom))
title('dzcom')

%%%%%%%%%%%%%
% XCOM
bv0 = [xcom_human(1) xcom_human(1) xcom_human(20) xcom_human(end) xcom_human(end)];
fun = @(x) costFcnBezier(x,time_norm,xcom_human);
bv_xcom = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,4); hold on; grid on;
plot(time_norm,xcom_human)
plot(time_norm,evalBezier(bv_xcom,time_norm))
title('xcom')

bv0 = [dxcom_human(1) dxcom_human(1) dxcom_human(20) dxcom_human(end) dxcom_human(end)];
fun = @(x) costFcnBezier(x,time_norm,dxcom_human);
bv_dxcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,5); hold on; grid on;
plot(time_norm,dxcom_human)
plot(time_norm,evalBezier(bv_dxcom,time_norm))
title('dxcom')

bv0 = [ddxcom_human(1) ddxcom_human(1) ddxcom_human(20) ddxcom_human(end) ddxcom_human(end)];
fun = @(x) costFcnBezier(x,time_norm,ddxcom_human);
bv_ddxcom = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,6); hold on; grid on;
plot(time_norm,ddxcom_human)
plot(time_norm,evalBezier(bv_ddxcom,time_norm))
title('dxcom')

%%%%%%%%%%%%%
% GRF
bv0 = [grf_human_st(1) grf_human_st(1) grf_human_st(20) grf_human_st(end) grf_human_st(end)];
fun = @(x) costFcnBezier(x,time_norm,grf_human_st);
bv_grf_st = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,7); hold on; grid on;
plot(time_norm,grf_human_st)
plot(time_norm,evalBezier(bv_grf_st,time_norm))
title('grf st')

bv0 = [grf_human_sw(1) grf_human_sw(1) grf_human_sw(20) grf_human_sw(end) grf_human_sw(end)];
fun = @(x) costFcnBezier(x,time_norm,grf_human_sw);
bv_grf_sw = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,8); hold on; grid on;
plot(time_norm,grf_human_sw)
plot(time_norm,evalBezier(bv_grf_sw,time_norm))
title('grf sw')

bv0 = [grf_human_SSP(1) grf_human_SSP(1) grf_human_SSP(20) grf_human_SSP(end) grf_human_SSP(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_human_SSP);
bv_grf_SSP = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,9); hold on; grid on;
plot(time_norm_SSP,grf_human_SSP)
plot(time_norm_SSP,evalBezier(bv_grf_SSP,time_norm_SSP))
title('grf SSP')

bv0 = [grf_human_DSP_st(1) grf_human_DSP_st(1) grf_human_DSP_st(20) grf_human_DSP_st(end) grf_human_DSP_st(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_human_DSP_st);
bv_grf_DSP_st = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,10); hold on; grid on;
plot(time_norm_SSP,grf_human_DSP_st)
plot(time_norm_SSP,evalBezier(bv_grf_DSP_st,time_norm_SSP))
title('grf DSP_{st}')

bv0 = [grf_human_DSP_sw(1) grf_human_DSP_sw(1) grf_human_DSP_sw(20) grf_human_DSP_sw(end) grf_human_DSP_sw(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_human_DSP_sw);
bv_grf_DSP_sw = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,11); hold on; grid on;
plot(time_norm_SSP,grf_human_DSP_sw)
plot(time_norm_SSP,evalBezier(bv_grf_DSP_sw,time_norm_SSP))
title('grf DSP_{sw}')

%%%%%%%%%%%%
% dGRF
bv0 = [dgrf_human_st(1) dgrf_human_st(1) dgrf_human_st(20) dgrf_human_st(end) dgrf_human_st(end)];
fun = @(x) costFcnBezier(x,time_norm,dgrf_human_st);
bv_dgrf_st = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,7); hold on; grid on;
% plot(time_norm,dgrf_human_st)
% plot(time_norm,evalBezier(bv_dgrf_st,time_norm))
% title('dgrf st')

bv0 = [dgrf_human_sw(1) dgrf_human_sw(1) dgrf_human_sw(20) dgrf_human_sw(end) dgrf_human_sw(end)];
fun = @(x) costFcnBezier(x,time_norm,dgrf_human_sw);
bv_dgrf_sw = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,8); hold on; grid on;
% plot(time_norm,dgrf_human_sw)
% plot(time_norm,evalBezier(bv_dgrf_sw,time_norm))
% title('dgrf sw')

bv0 = [dgrf_human_SSP(1) dgrf_human_SSP(1) dgrf_human_SSP(20) dgrf_human_SSP(end) dgrf_human_SSP(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_human_SSP);
bv_dgrf_SSP = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,9); hold on; grid on;
% plot(time_norm_SSP,dgrf_human_SSP)
% plot(time_norm_SSP,evalBezier(bv_dgrf_SSP,time_norm_SSP))
% title('dgrf SSP')

bv0 = [dgrf_human_DSP_st(1) dgrf_human_DSP_st(1) dgrf_human_DSP_st(20) dgrf_human_DSP_st(end) dgrf_human_DSP_st(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_human_DSP_st);
bv_dgrf_DSP_st = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,10); hold on; grid on;
% plot(time_norm_SSP,dgrf_human_DSP_st)
% plot(time_norm_SSP,evalBezier(bv_dgrf_DSP_st,time_norm_SSP))
% title('dgrf DSP_{st}')

bv0 = [dgrf_human_DSP_sw(1) dgrf_human_DSP_sw(1) dgrf_human_DSP_sw(20) dgrf_human_DSP_sw(end) dgrf_human_DSP_sw(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_human_DSP_sw);
bv_dgrf_DSP_sw = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,11); hold on; grid on;
% plot(time_norm_SSP,dgrf_human_DSP_sw)
% plot(time_norm_SSP,evalBezier(bv_dgrf_DSP_sw,time_norm_SSP))
% title('dgrf DSP_{sw}')

%% Save Human splines to .mat file
nominalBeziers.bv_zcom = bv_zcom;
nominalBeziers.bv_dzcom = bv_dzcom;
nominalBeziers.bv_ddzcom = bv_ddzcom;
nominalBeziers.bv_xcom = bv_xcom;
nominalBeziers.bv_dxcom = bv_dxcom;
nominalBeziers.bv_ddxcom = bv_ddxcom;
nominalBeziers.bv_grf_st = bv_grf_st;
nominalBeziers.bv_grf_sw = bv_grf_sw;
nominalBeziers.bv_grf_SSP = bv_grf_SSP;
nominalBeziers.bv_grf_DSP_st = bv_grf_DSP_st;
nominalBeziers.bv_grf_DSP_sw = bv_grf_DSP_sw;
nominalBeziers.bv_dgrf_st = bv_dgrf_st;
nominalBeziers.bv_dgrf_sw = bv_dgrf_sw;
nominalBeziers.bv_dgrf_SSP = bv_dgrf_SSP;
nominalBeziers.bv_dgrf_DSP_st = bv_dgrf_DSP_st;
nominalBeziers.bv_dgrf_DSP_sw = bv_dgrf_DSP_sw;
nominalBeziers.xcomMin = xcom_human(1);
nominalBeziers.xcomMax = xcom_human(end);
nominalBeziers.timeMin = time_human(1);
nominalBeziers.timeMax = time_human(end);
nominalBeziers.timeMax_SSP = time_human_SSP(end);
save('data/outputs/nominalBeziersHuman.mat','nominalBeziers')

%% Create VLO to VLO data for downstep comparison
[~,idxVLO] = max(SSP1sol.z);
%%% STEPDOWN
time_human_sd = [SSP1sol.t(idxVLO:end) SSP1sol.t(end)+DSP1sol.t(2:end)]...
                 - SSP1sol.t(idxVLO);
time_norm_sd = time_human_sd./(time_human_sd(end));

time_human_SSP_sd = SSP1sol.t(idxVLO:end) - SSP1sol.t(idxVLO);
time_norm_SSP_sd = time_human_SSP_sd./(time_human_SSP_sd(end));
time_human_DSP_sd = DSP1sol.t(2:end) - DSP1sol.t(2);
time_norm_DSP_sd = time_human_DSP_sd./(time_human_DSP_sd(end));

xcom_human_sd = [SSP1sol.x(idxVLO:end) DSP1sol.x(2:end)];
dxcom_human_sd = [SSP1sol.dx(idxVLO:end) DSP1sol.dx(2:end)];
ddxcom_human_sd = [SSP1sol.ddx(idxVLO:end) DSP1sol.ddx(2:end)];

zcom_human_sd = [SSP1sol.z(idxVLO:end) DSP1sol.z(2:end)] - 0.01;
dzcom_human_sd = [SSP1sol.dz(idxVLO:end) DSP1sol.dz(2:end)];
ddzcom_human_sd = [SSP1sol.ddz(idxVLO:end) DSP1sol.ddz(2:end)];

grf_human_st_sd = [SSP1sol.GRF(1,idxVLO:end) DSP1sol.leg1.GRF(1,2:end)];
grf_human_sw_sd = [SSP1sol.GRF(2,idxVLO:end) DSP1sol.leg1.GRF(2,2:end)];
grf_human_SSP_sd = SSP1sol.GRF(1,idxVLO:end);
grf_human_DSP_st_sd = DSP1sol.leg1.GRF(1,2:end);
grf_human_DSP_sw_sd = DSP1sol.leg1.GRF(2,2:end);

%%% STEPUP
time_human_su = SSP1sol.t(1:idxVLO);
time_norm_su = time_human_su./(time_human_su(end));

time_human_SSP_su = SSP1sol.t(1:idxVLO);
time_norm_SSP_su = time_human_SSP_su./(time_human_SSP_su(end));

xcom_human_su = SSP1sol.x(1:idxVLO);
dxcom_human_su = SSP1sol.dx(1:idxVLO);
ddxcom_human_su = SSP1sol.ddx(1:idxVLO);

zcom_human_su = SSP1sol.z(1:idxVLO);
dzcom_human_su = SSP1sol.dz(1:idxVLO);
ddzcom_human_su = SSP1sol.ddz(1:idxVLO);

grf_human_st_su = SSP1sol.GRF(1,1:idxVLO);
grf_human_sw_su = SSP1sol.GRF(2,1:idxVLO);
grf_human_SSP_su = SSP1sol.GRF(1,1:idxVLO);

%% Save  Human VLO2VLO Data to .mat file
downstepDataHumanNominal.exp00.phase1.time = time_human_sd;
downstepDataHumanNominal.exp00.phase1.time_SSP = time_human_SSP_sd;
downstepDataHumanNominal.exp00.phase1.time_DSP = time_human_DSP_sd;
downstepDataHumanNominal.exp00.phase1.time_norm = time_norm_sd;
downstepDataHumanNominal.exp00.phase1.time_norm_SSP = time_norm_SSP_sd;
downstepDataHumanNominal.exp00.phase1.time_norm_DSP = time_norm_DSP_sd;
downstepDataHumanNominal.exp00.phase1.zcom = zcom_human_sd;
downstepDataHumanNominal.exp00.phase1.dzcom = dzcom_human_sd;
downstepDataHumanNominal.exp00.phase1.ddzcom = ddzcom_human_sd;
downstepDataHumanNominal.exp00.phase1.xcom = xcom_human_sd;
downstepDataHumanNominal.exp00.phase1.dxcom = dxcom_human_sd;
downstepDataHumanNominal.exp00.phase1.ddxcom = ddxcom_human_sd;
downstepDataHumanNominal.exp00.phase1.grf_st = grf_human_st_sd;
downstepDataHumanNominal.exp00.phase1.grf_sw = grf_human_sw_sd;
downstepDataHumanNominal.exp00.phase1.grf_SSP = grf_human_SSP_sd;
downstepDataHumanNominal.exp00.phase1.grf_DSP_st = grf_human_DSP_st_sd;
downstepDataHumanNominal.exp00.phase1.grf_DSP_sw = grf_human_DSP_sw_sd;

downstepDataHumanNominal.exp00.phase2.time = time_human;
downstepDataHumanNominal.exp00.phase2.time_SSP = time_human_SSP;
downstepDataHumanNominal.exp00.phase2.time_DSP = time_human_DSP;
downstepDataHumanNominal.exp00.phase2.time_norm = time_norm;
downstepDataHumanNominal.exp00.phase2.time_norm_SSP = time_norm_SSP;
downstepDataHumanNominal.exp00.phase2.time_norm_DSP = time_norm_DSP;
downstepDataHumanNominal.exp00.phase2.zcom = zcom_human;
downstepDataHumanNominal.exp00.phase2.dzcom = dzcom_human;
downstepDataHumanNominal.exp00.phase2.ddzcom = ddzcom_human;
downstepDataHumanNominal.exp00.phase2.xcom = xcom_human;
downstepDataHumanNominal.exp00.phase2.dxcom = dxcom_human;
downstepDataHumanNominal.exp00.phase2.ddxcom = ddxcom_human;
downstepDataHumanNominal.exp00.phase2.grf_st = grf_human_st;
downstepDataHumanNominal.exp00.phase2.grf_sw = grf_human_sw;
downstepDataHumanNominal.exp00.phase2.grf_SSP = grf_human_SSP;
downstepDataHumanNominal.exp00.phase2.grf_DSP_st = grf_human_DSP_st;
downstepDataHumanNominal.exp00.phase2.grf_DSP_sw = grf_human_DSP_sw;

downstepDataHumanNominal.exp00.phase3.time = time_human_su;
downstepDataHumanNominal.exp00.phase3.time_SSP = time_human_SSP_su;
downstepDataHumanNominal.exp00.phase3.time_DSP = [0];
downstepDataHumanNominal.exp00.phase3.time_norm = time_norm_su;
downstepDataHumanNominal.exp00.phase3.time_norm_SSP = time_norm_SSP_su;
downstepDataHumanNominal.exp00.phase3.time_norm_DSP = [0];
downstepDataHumanNominal.exp00.phase3.zcom = zcom_human_su;
downstepDataHumanNominal.exp00.phase3.dzcom = dzcom_human_su;
downstepDataHumanNominal.exp00.phase3.ddzcom = ddzcom_human_su;
downstepDataHumanNominal.exp00.phase3.xcom = xcom_human_su;
downstepDataHumanNominal.exp00.phase3.dxcom = dxcom_human_su;
downstepDataHumanNominal.exp00.phase3.ddxcom = ddxcom_human_su;
downstepDataHumanNominal.exp00.phase3.grf_st = grf_human_st_su;
downstepDataHumanNominal.exp00.phase3.grf_sw = grf_human_sw_su;
downstepDataHumanNominal.exp00.phase3.grf_SSP = grf_human_SSP_su;
downstepDataHumanNominal.exp00.phase3.grf_DSP_st = [0];
downstepDataHumanNominal.exp00.phase3.grf_DSP_sw = [0];

nominalDataHumanAsDownstep = downstepDataHumanNominal;
save('data/beziers/nominalDataHumanAsDownstep.mat','nominalDataHumanAsDownstep')


%% Create VLO to VLO bezier splines for downstep comparison
mid = round(length(zcom_human_sd)/2);
%%% make splines for this
bv0 = [zcom_human_sd(1) zcom_human_sd(1) zcom_human_sd(mid) zcom_human_sd(end) zcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,zcom_human_sd);
bv_zcom_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [dzcom_human_sd(1) dzcom_human_sd(1) dzcom_human_sd(mid) dzcom_human_sd(end) dzcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,dzcom_human_sd);
bv_dzcom_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [ddzcom_human_sd(1) ddzcom_human_sd(1) ddzcom_human_sd(mid) ddzcom_human_sd(end) ddzcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,ddzcom_human_sd);
bv_ddzcom_sd = fmincon(fun,bv0,[],[],[],[])

%%%%%%%%%%%%%
% XCOM
bv0 = [xcom_human_sd(1) xcom_human_sd(1) xcom_human_sd(mid) xcom_human_sd(end) xcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,xcom_human_sd);
bv_xcom_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [dxcom_human_sd(1) dxcom_human_sd(1) dxcom_human_sd(mid) dxcom_human_sd(end) dxcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,dxcom_human_sd);
bv_dxcom_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [ddxcom_human_sd(1) ddxcom_human_sd(1) ddxcom_human_sd(mid) ddxcom_human_sd(end) ddxcom_human_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,ddxcom_human_sd);
bv_ddxcom_sd = fmincon(fun,bv0,[],[],[],[])

%%%%%%%%%%%%%
% GRF
bv0 = [grf_human_st_sd(1) grf_human_st_sd(1) grf_human_st_sd(mid) grf_human_st_sd(end) grf_human_st_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,grf_human_st_sd);
% bv_grf_st_sd = fmincon(fun,bv0,[],[],[1 -1 0 0 0],[0])
bv_grf_st_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_sw_sd(1) grf_human_sw_sd(1) grf_human_sw_sd(mid) grf_human_sw_sd(end) grf_human_sw_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_sd,grf_human_sw_sd);
bv_grf_sw_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_SSP_sd(1) grf_human_SSP_sd(1) grf_human_SSP_sd(5) grf_human_SSP_sd(end) grf_human_SSP_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP_sd,grf_human_SSP_sd);
% bv_grf_SSP_sd = fmincon(fun,bv0,[],[],[0 0 0 1 -1],[0])
bv_grf_SSP_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_DSP_st_sd(1) grf_human_DSP_st_sd(1) grf_human_DSP_st_sd(5) grf_human_DSP_st_sd(end) grf_human_DSP_st_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_DSP_sd,grf_human_DSP_st_sd);
% bv_grf_DSP_st_sd = fmincon(fun,bv0,[],[],[1 -1 0 0 0],[0])
bv_grf_DSP_st_sd = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_DSP_sw_sd(1) grf_human_DSP_sw_sd(1) grf_human_DSP_sw_sd(5) grf_human_DSP_sw_sd(end) grf_human_DSP_sw_sd(end)];
fun = @(x) costFcnBezier(x,time_norm_DSP_sd,grf_human_DSP_sw_sd);
% bv_grf_DSP_sw_sd = fmincon(fun,bv0,[],[],[0 0 0 1 -1],[0])
bv_grf_DSP_sw_sd = fmincon(fun,bv0,[],[],[],[])

n = 0:0.01:1;
figure
subplot(1,5,1)
plot(n,bezier2(bv_grf_st_sd,n))
subplot(1,5,2)
plot(n,bezier2(bv_grf_sw_sd,n))
subplot(1,5,3)
plot(n,bezier2(bv_grf_SSP_sd,n))
subplot(1,5,4)
plot(n,bezier2(bv_grf_DSP_st_sd,n))
subplot(1,5,5)
plot(n,bezier2(bv_grf_DSP_sw_sd,n))

mid = round(length(zcom_human_su)/2);
bv0 = [zcom_human_su(1) zcom_human_su(1) zcom_human_su(mid) zcom_human_su(end) zcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,zcom_human_su);
bv_zcom_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [dzcom_human_su(1) dzcom_human_su(1) dzcom_human_su(mid) dzcom_human_su(end) dzcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,dzcom_human_su);
bv_dzcom_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [ddzcom_human_su(1) ddzcom_human_su(1) ddzcom_human_su(mid) ddzcom_human_su(end) ddzcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,ddzcom_human_su);
bv_ddzcom_su = fmincon(fun,bv0,[],[],[],[])

%%%%%%%%%%%%%
% XCOM
bv0 = [xcom_human_su(1) xcom_human_su(1) xcom_human_su(mid) xcom_human_su(end) xcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,xcom_human_su);
bv_xcom_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [dxcom_human_su(1) dxcom_human_su(1) dxcom_human_su(mid) dxcom_human_su(end) dxcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,dxcom_human_su);
bv_dxcom_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [ddxcom_human_su(1) ddxcom_human_su(1) ddxcom_human_su(mid) ddxcom_human_su(end) ddxcom_human_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,ddxcom_human_su);
bv_ddxcom_su = fmincon(fun,bv0,[],[],[],[])

%%%%%%%%%%%%%
% GRF
bv0 = [grf_human_st_su(1) grf_human_st_su(1) grf_human_st_su(mid) grf_human_st_su(end) grf_human_st_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,grf_human_st_su);
% bv_grf_st_su = fmincon(fun,bv0,[],[],[1 -1 0 0 0; 0 0 0 1 -1],[0;0])
bv_grf_st_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_sw_su(1) grf_human_sw_su(1) grf_human_sw_su(mid) grf_human_sw_su(end) grf_human_sw_su(end)];
fun = @(x) costFcnBezier(x,time_norm_su,grf_human_sw_su);
bv_grf_sw_su = fmincon(fun,bv0,[],[],[],[])

bv0 = [grf_human_SSP_su(1) grf_human_SSP_su(1) grf_human_SSP_su(5) grf_human_SSP_su(end) grf_human_SSP_su(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP_su,grf_human_SSP_su);
% bv_grf_SSP_su = fmincon(fun,bv0,[],[],[1 -1 0 0 0; 0 0 0 1 -1],[0;0])
bv_grf_SSP_su = fmincon(fun,bv0,[],[],[],[])

n = 0:0.01:1;
figure
subplot(1,5,1)
plot(n,bezier2(bv_grf_st_su,n))
subplot(1,5,2)
plot(n,bezier2(bv_grf_sw_su,n))
subplot(1,5,3)
plot(n,bezier2(bv_grf_SSP_su,n))

%% Save  Human VLO2VLO splines to .mat file
downstepBeziersHumanNominal.exp00.phase1.bv_zcom = bv_zcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_dzcom = bv_dzcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_ddzcom = bv_ddzcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_xcom = bv_xcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_dxcom = bv_dxcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_ddxcom = bv_ddxcom_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_grf_st = bv_grf_st_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_grf_sw = bv_grf_sw_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_grf_SSP = bv_grf_SSP_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_grf_DSP_st = bv_grf_DSP_st_sd;
downstepBeziersHumanNominal.exp00.phase1.bv_grf_DSP_sw = bv_grf_DSP_sw_sd;
downstepBeziersHumanNominal.exp00.phase1.timeMax = time_human_sd(end);
downstepBeziersHumanNominal.exp00.phase1.timeMax_SSP = time_human_SSP_sd(end);
downstepBeziersHumanNominal.exp00.phase1.xcomMin = xcom_human_sd(1);
downstepBeziersHumanNominal.exp00.phase1.xcomMax = xcom_human_sd(end);
downstepBeziersHumanNominal.exp00.phase1.timeMin = time_human_sd(1);

downstepBeziersHumanNominal.exp00.phase2.bv_zcom = bv_zcom;
downstepBeziersHumanNominal.exp00.phase2.bv_dzcom = bv_dzcom;
downstepBeziersHumanNominal.exp00.phase2.bv_ddzcom = bv_ddzcom;
downstepBeziersHumanNominal.exp00.phase2.bv_xcom = bv_xcom;
downstepBeziersHumanNominal.exp00.phase2.bv_dxcom = bv_dxcom;
downstepBeziersHumanNominal.exp00.phase2.bv_ddxcom = bv_ddxcom;
downstepBeziersHumanNominal.exp00.phase2.bv_grf_st = bv_grf_st;
downstepBeziersHumanNominal.exp00.phase2.bv_grf_sw = bv_grf_sw;
downstepBeziersHumanNominal.exp00.phase2.bv_grf_SSP = bv_grf_SSP;
downstepBeziersHumanNominal.exp00.phase2.bv_grf_DSP_st = bv_grf_DSP_st;
downstepBeziersHumanNominal.exp00.phase2.bv_grf_DSP_sw = bv_grf_DSP_sw;
downstepBeziersHumanNominal.exp00.phase2.timeMax = time_human(end);
downstepBeziersHumanNominal.exp00.phase2.timeMax_SSP = time_human_SSP(end);
downstepBeziersHumanNominal.exp00.phase2.xcomMin = xcom_human(1);
downstepBeziersHumanNominal.exp00.phase2.xcomMax = xcom_human(end);
downstepBeziersHumanNominal.exp00.phase2.timeMin = time_human(1);

downstepBeziersHumanNominal.exp00.phase3.bv_zcom = bv_zcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_dzcom = bv_dzcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_ddzcom = bv_ddzcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_xcom = bv_xcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_dxcom = bv_dxcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_ddxcom = bv_ddxcom_su;
downstepBeziersHumanNominal.exp00.phase3.bv_grf_st = bv_grf_st_su;
downstepBeziersHumanNominal.exp00.phase3.bv_grf_sw = bv_grf_sw_su;
downstepBeziersHumanNominal.exp00.phase3.bv_grf_SSP = bv_grf_SSP_su;
downstepBeziersHumanNominal.exp00.phase3.timeMax = time_human_su(end);
downstepBeziersHumanNominal.exp00.phase3.timeMax_SSP = time_human_SSP_su(end);
downstepBeziersHumanNominal.exp00.phase3.xcomMin = xcom_human_su(1);
downstepBeziersHumanNominal.exp00.phase3.xcomMax = xcom_human_su(end);
downstepBeziersHumanNominal.exp00.phase3.timeMin = time_human_su(1);

nominalBeziersHumanAsDownstep = downstepBeziersHumanNominal;
save('data/beziers/nominalBeziersHumanAsDownstep.mat','nominalBeziersHumanAsDownstep')





%% Scaling parameters
g = 9.81;

delta = 1 - (0.25/(xcom_human(end) - xcom_human(1)));
L_human = sqrt(zcom_human.^2 + (delta*xcom_human).^2);
dL_human = (delta^2*xcom_human.*dxcom_human + zcom_human.*dzcom_human)./L_human;
ddL_human = (2*delta^2*xcom_human.*ddxcom_human + 2*delta^2.*dxcom_human.^2 + 2*zcom_human.*ddzcom_human + 2*dzcom_human.^2)./(2*L_human) - ...
    (2*delta^2*xcom_human.*dxcom_human + 2*zcom_human.*dzcom_human)./((4*(delta^2*xcom_human.^2 + zcom_human.^2)).^(3/2));

th_human = atan2(zcom_human,delta*xcom_human);
dth_human = (delta*(xcom_human.*dzcom_human - zcom_human.*dxcom_human))./(delta^2*xcom_human.^2 + zcom_human.^2);
ddth_human = 1./(L_human.^2).*delta.*(xcom_human.*zcom_human.*(2*delta^2*dxcom_human.^2 + zcom_human.*ddzcom_human-2*dzcom_human.^2) - ...
                                      delta^2*xcom_human.^2.*(zcom_human.*ddxcom_human + 2.*dxcom_human.*dzcom_human) + ...
                                      delta^2.*xcom_human.^3.*ddzcom_human + zcom_human.^2.*(2.*dxcom_human.*dzcom_human - zcom_human.*ddxcom_human));

L_scaling = 0.90;
L_cassie = L_human*L_scaling;
dL_cassie = dL_human*L_scaling;
ddL_cassie = ddL_human*L_scaling;
th_cassie = th_human;
dth_cassie = dth_human;
ddth_cassie = ddth_human;

xcom_cassie = L_cassie.*cos(th_cassie);
dxcom_cassie = dL_cassie.*cos(th_cassie) - L_cassie.*sin(th_cassie).*dth_cassie;
ddxcom_cassie = ddL_cassie.*cos(th_cassie) - dL_cassie.*sin(th_cassie).*dth_cassie - ...
                dL_cassie.*sin(th_cassie).*dth_cassie - L_cassie.*cos(th_cassie).*dth_cassie.^2 - ...
                L_cassie.*sin(th_cassie).*ddth_cassie;

zcom_cassie = L_cassie.*sin(th_cassie);
dzcom_cassie = dL_cassie.*sin(th_cassie) + L_cassie.*cos(th_cassie).*dth_cassie;
ddzcom_cassie = ddL_cassie.*sin(th_cassie) + dL_cassie.*cos(th_cassie).*dth_cassie + ...
                dL_cassie.*cos(th_cassie).*dth_cassie - L_cassie.*sin(th_cassie).*dth_cassie.^2 + ...
                L_cassie.*cos(th_cassie).*ddth_cassie;          
            
% scaling to lmit downward velocity
x_scaling = (xcom_cassie(end)-xcom_cassie(1))/(xcom_human(end) - xcom_human(1));            
% zcom_cassie = (zcom_cassie - mean(zcom_cassie))*x_scaling + mean(zcom_cassie);
% dzcom_cassie = dzcom_cassie*x_scaling;
% ddzcom_cassie = ddzcom_cassie*x_scaling;

figure; 
subplot(1,3,1); hold on; grid on;
plot(xcom_human,zcom_human)
plot(xcom_cassie,zcom_cassie)
subplot(1,3,2); hold on; grid on;
plot(dxcom_human,dzcom_human)
plot(dxcom_cassie,dzcom_cassie)
subplot(1,3,3); hold on; grid on;
plot(ddxcom_human,ddzcom_human)
plot(ddxcom_cassie,ddzcom_cassie)

% reduce by 0.10 due to foot roll
stepLength_human = xcom_human(end) - xcom_human(1);
% stepLength_scaling = delta;
% stepLength_cassie = stepLength_scaling*stepLength_human;

Ts_scaling = ( sqrt(mean(L_cassie)/g) )/( sqrt(mean(L_human)/g) );
Ts_human = time_human(end)-time_human(1);
Ts_cassie = Ts_scaling*Ts_human;

HLIP_vxd = 0.8;
stepLength_cassie = HLIP_vxd*Ts_cassie;
% HLIP_vxd = stepLength_cassie/Ts_cassie;

%%%%%%%%%%%%%
% create scaling parameters

% x_scaling = stepLength_cassie / stepLength_human;
dx_scaling = HLIP_vxd / mean(dxcom_human);
ddx_scaling = dx_scaling;
xcom_cassie = x_scaling*xcom_human;
dxcom_cassie = dx_scaling*dxcom_human;
ddxcom_cassie = ddx_scaling*ddxcom_human;

stepLength_scaling = x_scaling;
z_scaling = 1;
dz_scaling = stepLength_scaling;
ddz_scaling = stepLength_scaling;

m_cassie = 31;
m_human = 66.5138;
grf_scaling = m_cassie / m_human;


% L_cassie = L_scaling.*L_human;
% Ls_cassie = Ts_cassie*HLIP_vxd;

x_scaling = stepLength_cassie / stepLength_human;
dx_scaling = HLIP_vxd / mean(dxcom_human);
ddx_scaling = dx_scaling;

scaling.stepLength_human = stepLength_human;
scaling.stepLength_cassie = stepLength_cassie;
scaling.stepLength_scaling = stepLength_scaling;
scaling.x_scaling = x_scaling;
scaling.dx_scaling = dx_scaling;
scaling.ddx_scaling = ddx_scaling;
scaling.z_scaling = z_scaling;
scaling.dz_scaling = dz_scaling;
scaling.ddz_scaling = ddz_scaling;
scaling.grf_scaling = grf_scaling;
scaling.Ts_scaling = Ts_scaling;
scaling.L_scaling = L_scaling;
% save('data/beziers/scaling.mat','scaling')





%% Create Cassie data
time_cassie = Ts_scaling*time_human;
time_cassie_SSP = Ts_scaling*time_human_SSP;

grf_cassie_st = grf_human_st*grf_scaling;
grf_cassie_sw = grf_human_sw*grf_scaling;
grf_cassie_SSP = grf_human_SSP*grf_scaling;
grf_cassie_DSP_st = grf_human_DSP_st*grf_scaling;
grf_cassie_DSP_sw = grf_human_DSP_sw*grf_scaling;

dgrf_cassie_sw = [0 diff(grf_cassie_st)./diff(time_cassie)];
dgrf_cassie_st = [diff(grf_cassie_sw)./diff(time_cassie) 0];
dgrf_cassie_SSP = [diff(grf_cassie_SSP)./diff(time_cassie_SSP) 0];
dgrf_cassie_DSP_st = [0 diff(grf_cassie_DSP_st)./diff(time_cassie_SSP)];
dgrf_cassie_DSP_sw = [diff(grf_cassie_DSP_sw)./diff(time_cassie_SSP) 0];

%% Save data to .mat file
nominalDataCassie.exp00.time = time_cassie;
nominalDataCassie.exp00.time_SSP = time_cassie_SSP;
nominalDataCassie.exp00.time_norm = time_norm;
nominalDataCassie.exp00.time_norm_SSP = time_norm_SSP;
nominalDataCassie.exp00.xcom = xcom_cassie;
nominalDataCassie.exp00.dxcom = dxcom_cassie;
nominalDataCassie.exp00.ddxcom = ddxcom_cassie;
nominalDataCassie.exp00.zcom = zcom_cassie;
nominalDataCassie.exp00.dzcom = dzcom_cassie;
nominalDataCassie.exp00.ddzcom = ddzcom_cassie;
nominalDataCassie.exp00.grf_st = grf_cassie_st;
nominalDataCassie.exp00.grf_sw = grf_cassie_sw;
nominalDataCassie.exp00.grf_SSP = grf_cassie_SSP;
nominalDataCassie.exp00.grf_DSP_st = grf_cassie_DSP_st;
nominalDataCassie.exp00.grf_DSP_sw = grf_cassie_DSP_sw;
% save('data/beziers/nominalDataCassie.mat','nominalDataCassie')

%%%%%%%%%%%%%%%%%%%%%%
%%% CASSIE SPLINES %%%
%%%%%%%%%%%%%%%%%%%%%%

%% Create Cassie splines
figure

%%%%%%%%%%%%%
% ZCOM
bv0 = [zcom_cassie(1) zcom_cassie(1) zcom_cassie(20) zcom_cassie(end) zcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm,zcom_cassie);
bv_zcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,1); hold on; grid on;
plot(time_norm,zcom_cassie)
plot(time_norm,evalBezier(bv_zcom,time_norm))
title('zcom')

bv0 = [dzcom_cassie(1) dzcom_cassie(1) dzcom_cassie(20) dzcom_cassie(end) dzcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm,dzcom_cassie);
bv_dzcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,2); hold on; grid on;
plot(time_norm,dzcom_cassie)
plot(time_norm,evalBezier(bv_dzcom,time_norm))
title('dzcom')

ddzcom_cassie = ddzcom_cassie(1:25);
time_norm_ddzcom = time_norm(1:25)/time_norm(25);
bv0 = [ddzcom_cassie(1) ddzcom_cassie(1) ddzcom_cassie(10) ddzcom_cassie(end) ddzcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm_ddzcom,ddzcom_cassie);
bv_ddzcom = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,3); hold on; grid on;
plot(time_norm_ddzcom,ddzcom_cassie)
plot(time_norm_ddzcom,evalBezier(bv_ddzcom,time_norm_ddzcom))
title('dzcom')

%%%%%%%%%%%%%
% XCOM
bv0 = [xcom_cassie(1) xcom_cassie(1) xcom_cassie(20) xcom_cassie(end) xcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm,xcom_cassie);
bv_xcom = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,4); hold on; grid on;
plot(time_norm,xcom_cassie)
plot(time_norm,evalBezier(bv_xcom,time_norm))
title('xcom')

bv0 = [dxcom_cassie(1) dxcom_cassie(1) dxcom_cassie(20) dxcom_cassie(end) dxcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm,dxcom_cassie);
bv_dxcom = fmincon(fun,bv0,[],[],[1 0 0 0 -1],[0])
subplot(4,3,5); hold on; grid on;
plot(time_norm,dxcom_cassie)
plot(time_norm,evalBezier(bv_dxcom,time_norm))
title('dxcom')

bv0 = [ddxcom_cassie(1) ddxcom_cassie(1) ddxcom_cassie(20) ddxcom_cassie(end) ddxcom_cassie(end)];
fun = @(x) costFcnBezier(x,time_norm,ddxcom_cassie);
bv_ddxcom = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,6); hold on; grid on;
plot(time_norm,ddxcom_cassie)
plot(time_norm,evalBezier(bv_ddxcom,time_norm))
title('dxcom')

%%%%%%%%%%%%%
% GRF
bv0 = [grf_cassie_st(1) grf_cassie_st(1) grf_cassie_st(20) grf_cassie_st(end) grf_cassie_st(end)];
fun = @(x) costFcnBezier(x,time_norm,grf_cassie_st);
bv_grf_st = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,7); hold on; grid on;
plot(time_norm,grf_cassie_st)
plot(time_norm,evalBezier(bv_grf_st,time_norm))
title('grf st')

bv0 = [grf_cassie_sw(1) grf_cassie_sw(1) grf_cassie_sw(20) grf_cassie_sw(end) grf_cassie_sw(end)];
fun = @(x) costFcnBezier(x,time_norm,grf_cassie_sw);
bv_grf_sw = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,8); hold on; grid on;
plot(time_norm,grf_cassie_sw)
plot(time_norm,evalBezier(bv_grf_sw,time_norm))
title('grf sw')

bv0 = [grf_cassie_SSP(1) grf_cassie_SSP(1) grf_cassie_SSP(20) grf_cassie_SSP(end) grf_cassie_SSP(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_cassie_SSP);
bv_grf_SSP = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,9); hold on; grid on;
plot(time_norm_SSP,grf_cassie_SSP)
plot(time_norm_SSP,evalBezier(bv_grf_SSP,time_norm_SSP))
title('grf SSP')

bv0 = [grf_cassie_DSP_st(1) grf_cassie_DSP_st(1) grf_cassie_DSP_st(20) grf_cassie_DSP_st(end) grf_cassie_DSP_st(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_cassie_DSP_st);
bv_grf_DSP_st = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,10); hold on; grid on;
plot(time_norm_SSP,grf_cassie_DSP_st)
plot(time_norm_SSP,evalBezier(bv_grf_DSP_st,time_norm_SSP))
title('grf DSP_{st}')

bv0 = [grf_cassie_DSP_sw(1) grf_cassie_DSP_sw(1) grf_cassie_DSP_sw(20) grf_cassie_DSP_sw(end) grf_cassie_DSP_sw(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,grf_cassie_DSP_sw);
bv_grf_DSP_sw = fmincon(fun,bv0,[],[],[],[])
subplot(4,3,11); hold on; grid on;
plot(time_norm_SSP,grf_cassie_DSP_sw)
plot(time_norm_SSP,evalBezier(bv_grf_DSP_sw,time_norm_SSP))
title('grf DSP_{sw}')

%%%%%%%%%%%%
% dGRF
bv0 = [dgrf_cassie_st(1) dgrf_cassie_st(1) dgrf_cassie_st(20) dgrf_cassie_st(end) dgrf_cassie_st(end)];
fun = @(x) costFcnBezier(x,time_norm,dgrf_cassie_st);
bv_dgrf_st = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,7); hold on; grid on;
% plot(time_norm,dgrf_cassie_st)
% plot(time_norm,evalBezier(bv_dgrf_st,time_norm))
% title('dgrf st')

bv0 = [dgrf_cassie_sw(1) dgrf_cassie_sw(1) dgrf_cassie_sw(20) dgrf_cassie_sw(end) dgrf_cassie_sw(end)];
fun = @(x) costFcnBezier(x,time_norm,dgrf_cassie_sw);
bv_dgrf_sw = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,8); hold on; grid on;
% plot(time_norm,dgrf_cassie_sw)
% plot(time_norm,evalBezier(bv_dgrf_sw,time_norm))
% title('dgrf sw')

bv0 = [dgrf_cassie_SSP(1) dgrf_cassie_SSP(1) dgrf_cassie_SSP(20) dgrf_cassie_SSP(end) dgrf_cassie_SSP(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_cassie_SSP);
bv_dgrf_SSP = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,9); hold on; grid on;
% plot(time_norm_SSP,dgrf_cassie_SSP)
% plot(time_norm_SSP,evalBezier(bv_dgrf_SSP,time_norm_SSP))
% title('dgrf SSP')

bv0 = [dgrf_cassie_DSP_st(1) dgrf_cassie_DSP_st(1) dgrf_cassie_DSP_st(20) dgrf_cassie_DSP_st(end) dgrf_cassie_DSP_st(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_cassie_DSP_st);
bv_dgrf_DSP_st = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,10); hold on; grid on;
% plot(time_norm_SSP,dgrf_cassie_DSP_st)
% plot(time_norm_SSP,evalBezier(bv_dgrf_DSP_st,time_norm_SSP))
% title('dgrf DSP_{st}')

bv0 = [dgrf_cassie_DSP_sw(1) dgrf_cassie_DSP_sw(1) dgrf_cassie_DSP_sw(20) dgrf_cassie_DSP_sw(end) dgrf_cassie_DSP_sw(end)];
fun = @(x) costFcnBezier(x,time_norm_SSP,dgrf_cassie_DSP_sw);
bv_dgrf_DSP_sw = fmincon(fun,bv0,[],[],[],[])
% subplot(4,3,11); hold on; grid on;
% plot(time_norm_SSP,dgrf_cassie_DSP_sw)
% plot(time_norm_SSP,evalBezier(bv_dgrf_DSP_sw,time_norm_SSP))
% title('dgrf DSP_{sw}')

%% Print Cassie splines to cpp file
% fid = fopen('/home/none/cassie_ws_2/src/cassie_controllers/include/cassie_controllers/generated/getNominalBeziers.hpp','w');
fid = fopen('/home/none/trash.hpp','w');

fprintf(fid,'#include <cassie_common_toolbox/bezier_tools.hpp>\n');
fprintf(fid,'#ifndef NOMINAL_BEZIERS_HPP\n');
fprintf(fid,'#define NOMINAL_BEZIERS_HPP\n\n\n');
fprintf(fid,'void SPLINES_BEZIER::getNominalBeziers(){\n');

printBezier(fid,bv_zcom,'bv_zcom');
printBezier(fid,bv_dzcom,'bv_dzcom');
printBezier(fid,bv_ddzcom,'bv_ddzcom');
printBezier(fid,bv_xcom,'bv_xcom');
printBezier(fid,bv_dxcom,'bv_dxcom');
printBezier(fid,bv_ddxcom,'bv_ddxcom');
printBezier(fid,bv_grf_st,'bv_grf_st');
printBezier(fid,bv_grf_sw,'bv_grf_sw');
printBezier(fid,bv_grf_SSP,'bv_grf_SSP');
printBezier(fid,bv_grf_DSP_st,'bv_grf_DSP_st');
printBezier(fid,bv_grf_DSP_sw,'bv_grf_DSP_sw');

fprintf(fid,'timeMax_nominal = %f;\n',time_cassie(end));
fprintf(fid,'timeMax_SSP_nominal = %f;\n',time_cassie_SSP(end));
fprintf(fid,'}\n');
fprintf(fid,'#endif\n');

%% Save  Cassie splines to .mat file
nominalBeziers.bv_zcom = bv_zcom;
nominalBeziers.bv_dzcom = bv_dzcom;
nominalBeziers.bv_ddzcom = bv_ddzcom;
nominalBeziers.bv_xcom = bv_xcom;
nominalBeziers.bv_dxcom = bv_dxcom;
nominalBeziers.bv_ddxcom = bv_ddxcom;
nominalBeziers.bv_grf_st = bv_grf_st;
nominalBeziers.bv_grf_sw = bv_grf_sw;
nominalBeziers.bv_grf_SSP = bv_grf_SSP;
nominalBeziers.bv_grf_DSP_st = bv_grf_DSP_st;
nominalBeziers.bv_grf_DSP_sw = bv_grf_DSP_sw;
nominalBeziers.bv_dgrf_st = bv_dgrf_st;
nominalBeziers.bv_dgrf_sw = bv_dgrf_sw;
nominalBeziers.bv_dgrf_SSP = bv_dgrf_SSP;
nominalBeziers.bv_dgrf_DSP_st = bv_dgrf_DSP_st;
nominalBeziers.bv_dgrf_DSP_sw = bv_dgrf_DSP_sw;
nominalBeziers.xcomMin = xcom_cassie(1);
nominalBeziers.xcomMax = xcom_cassie(end);
nominalBeziers.timeMin = time_cassie(1);
nominalBeziers.timeMax = time_cassie(end);
nominalBeziers.timeMax_SSP = time_cassie_SSP(end);
save('data/outputs/nominalBeziersCassie.mat','nominalBeziers')









%% FUNCTIONS
function evals = evalBezier(bv,tdata)
    evals = zeros(size(tdata));
    for i = 1:length(tdata)
        evals(i) = bezier2(bv,tdata(i));
    end
end

function printBezier(fid,data,name)
    fprintf(fid,'this->%s.resize(%i);\n',name,length(data));
    fprintf(fid,'this->%s << %f,\n',name,data(1));
    for i = 2:length(data)-1
        fprintf(fid,'%f,\n',data(i));
    end
    fprintf(fid,'%f;\n\n\n',data(end));
end
