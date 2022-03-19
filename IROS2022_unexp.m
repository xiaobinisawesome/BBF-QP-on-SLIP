clear all; 
close all; 
addpath('functions')

load('data/outputs/nominalBeziersHuman.mat')
teval = linspace(0,1,100);

fs = 20; % 18
lw = 3;

%% Surface plot zcom
system = 'Human';
load('data/human/bezierStepTimes.mat')
load('data/beziersRaw/allBeziersHuman.mat')
load('data/outputs/bezierZcomInterpolationHuman.mat')
stepTimes = bezierStepTimes;
ab = allBeziers;
bvFits = bezierZcomInterpolation;

hs = linspace(0,-0.15,31);
times = zeros(30,300);
datas = zeros(30,300);
hss = zeros(30,300);
% unexpected
for i = 1:length(hs)
    h = hs(i);
    
    T1 = polyval(stepTimes.unexp.Ts1,h);
    T2 = polyval(stepTimes.unexp.Ts2,h);
    T3 = polyval(stepTimes.unexp.Ts3,h);
    teval1 = linspace(0,1,round(250*T1));
    teval2 = linspace(0,1,round(250*T2));
    teval3 = linspace(0,1,round(250*T3));
    
    n = length(ab.unexp00.phase1.bv_zcom);
    bv_1 = zeros(1,n);
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase1').zcom{ii},h);
    end
    data1 = bezier2(bv_1,teval1);
    
    n = length(ab.unexp00.phase2.bv_zcom);
    bv_2 = zeros(1,n);
    for ii = 1:n
        bv_2(ii) = polyval(bvFits.unexp.('phase2').zcom{ii},h);
    end
    data2 = bezier2(bv_2,teval2);
    
    n = length(ab.unexp00.phase3.bv_zcom);
    bv_3 = zeros(1,n);
    for ii = 1:n
        bv_3(ii) = polyval(bvFits.unexp.('phase3').zcom{ii},h);
    end
    data3 = bezier2(bv_3,teval3);
    data = [data1 data2 data3];

    time1 = linspace(0,T1,round(250*T1));
    time2 = linspace(0,T2,round(250*T2));
    time3 = linspace(0,T3,round(250*T3));
    time = [time1 time2+time1(end) time3+time1(end)+time2(end)];
    
    datas(i,1:length(data)) = data;
    times(i,1:length(time)) = time;
    hss(i,1:length(time)) = h*ones(1,length(time));
end
times(times==0) = nan;
hss(hss==0) = nan;
datas(datas==0) = nan;


times_zcom = times;
hss_zcom = hss;
datas_zcom = datas;

idx = 2;
times_zcom_loop = times_zcom(idx,:);
times_zcom_loop = times_zcom_loop(~isnan(times_zcom_loop));
hss_zcom_loop = hss_zcom(idx,:);
hss_zcom_loop = hss_zcom_loop(~isnan(hss_zcom_loop));
datas_zcom_loop = datas_zcom(idx,:);
datas_zcom_loop = datas_zcom_loop(~isnan(datas_zcom_loop));

times_zcom_loop_0 = times_zcom(idx,:);
times_zcom_loop_0 = times_zcom_loop_0(~isnan(times_zcom_loop_0));
hss_zcom_loop_0 = hss_zcom(idx,:);
hss_zcom_loop_0 = hss_zcom_loop_0(~isnan(hss_zcom_loop_0));
datas_zcom_loop_0 = datas_zcom(idx,:);
datas_zcom_loop_0 = datas_zcom_loop_0(~isnan(datas_zcom_loop_0));

idx = 20;
times_zcom_loop_1 = times_zcom(idx,:);
times_zcom_loop_1 = times_zcom_loop_1(~isnan(times_zcom_loop_1));
hss_zcom_loop_1 = hss_zcom(idx,:);
hss_zcom_loop_1 = hss_zcom_loop_1(~isnan(hss_zcom_loop_1));
datas_zcom_loop_1 = datas_zcom(idx,:);
datas_zcom_loop_1 = datas_zcom_loop_1(~isnan(datas_zcom_loop_1));

idxStart = 45;
idxEnd = 81;

times_zcom_loop = [times_zcom_loop_0(1:idxStart) ...
                   linspace(times_zcom_loop_0(idxStart),times_zcom_loop_1(idxEnd),idxEnd-idxStart) ...
                   times_zcom_loop_1(idxEnd:end)];
hss_zcom_loop   = [hss_zcom_loop_0(1:idxStart) ...
                   linspace(hss_zcom_loop_0(idxStart),hss_zcom_loop_1(idxEnd),idxEnd-idxStart) ...
                   hss_zcom_loop_1(idxEnd:end)];
datas_zcom_loop = [datas_zcom_loop_0(1:idxStart) ...
                   linspace(datas_zcom_loop_0(idxStart),datas_zcom_loop_1(idxEnd),idxEnd-idxStart) ...
                   datas_zcom_loop_1(idxEnd:end)];

times_zcom_loop = times_zcom_loop(1:262);
hss_zcom_loop = hss_zcom_loop(1:262);
datas_zcom_loop = datas_zcom_loop(1:262);

%% Get the duration to align with the SSP
t0s = linspace(0.2065,0.3426,length(hs));
tfs = linspace(0.7823,0.8430,length(hs));
t0 = t0s(idx);
tf = tfs(idx);

[~,idx0s] = find(times_zcom_loop>=t0,1);
[~,idxfs] = find(times_zcom_loop>=tf,1);
n_loop = idxfs - idx0s;


%% Surface plot Fdes
system = 'Human';
load('data/human/bezierStepTimes.mat')
load('data/beziersRaw/allBeziersHuman.mat')
load('data/outputs/bezierFInterpolationHuman.mat')
stepTimes = bezierStepTimes;
ab = allBeziers;
bvFits = bezierFInterpolation;

times = zeros(30,300);
datas = zeros(30,300);
hss = zeros(30,300);
% unexpected
for i = 1:length(hs)
    h = hs(i);
    
    TTotal = tf - t0;
    T1 = 0.15*TTotal;
    T2 = 0.70*TTotal;
    T3 = 0.15*TTotal;
    teval1 = linspace(0,1,round(0.15*n_loop));
    teval2 = linspace(0,1,round(0.70*n_loop));
    teval3 = linspace(0,1,round(0.15*n_loop));
    
    n = 4;
    bv_1 = zeros(1,n);
    for ii = 1:n
        bv_1(ii) = polyval(bvFits.unexp.('phase1').grf_DSP_sw{ii},h);
    end
    data1 = bezier2(bv_1,teval1);
    
    bv_2 = zeros(1,n);
    n = 5;
    for ii = 1:n
        bv_2(ii) = polyval(bvFits.unexp.('phase2').grf_SSP{ii},h);
    end
    data2 = bezier2(bv_2,teval2);
    
    bv_3 = zeros(1,n);
    n = 4;
    for ii = 1:n
        bv_3(ii) = polyval(bvFits.unexp.('phase2').grf_DSP_st{ii},h);
    end
    data3 = bezier2(bv_3,teval3);
    data = [data1 data2 data3];
    
    time1 = linspace(0,T1,round(0.15*n_loop));
    time2 = linspace(0,T2,round(0.70*n_loop));
    time3 = linspace(0,T3,round(0.15*n_loop));
    time = [time1 time2+time1(end) time3+time1(end)+time2(end)];
    
    datas(i,1:length(data)) = data;
    times(i,1:length(time)) = time;
    hss(i,1:length(time)) = h*ones(1,length(time));
end
times(times==0) = nan;
hss(hss==0) = nan;
datas(datas==0) = nan;


for i = 1:length(t0s)
    times(i,:) = times(i,:) + t0s(i);
end
times_fz = times;
hss_fz = hss;
datas_fz = datas;

times_fz_loop = times_fz(idx,:);
times_fz_loop = times_fz_loop(~isnan(times_fz_loop));
hss_fz_loop = hss_fz(idx,:);
hss_fz_loop = hss_fz_loop(~isnan(hss_fz_loop));
datas_fz_loop = datas_fz(idx,:);
datas_fz_loop = datas_fz_loop(~isnan(datas_fz_loop));

%% Plotting
% figure; 
% subplot(1,9,[1 2 3 4]); hold on; grid on;
% mesh(times_zcom,hss_zcom,datas_zcom)
% ylabel('h (m)','interpreter','latex')
% xlabel('t (s)','interpreter','latex')
% zlabel('$z_{CoM}^d$ (m)','interpreter','latex')
% set(gca,'FontSize',fs)
% % title('z_{CoM}^d(t,h)','Interpreter','tex')
% view([2 -3 1])
% 
% subplot(1,9,[6 7 8 9]); hold on; grid on;
% mesh(times_fz,hss_fz,datas_fz)
% ylabel('h (m)','interpreter','latex')
% xlabel('t (s)','interpreter','latex')
% zlabel('$F_z^d$ (N)','interpreter','latex')
% set(gca,'FontSize',fs)
% % title('F_{Z}^d(t,h)','Interpreter','tex')
% view([2 -3 1])

%% Animation
% v = VideoWriter('unexp_surface.avi','Motion JPEG AVI')
% open(v);

f = figure;

cnt_fz = 1;
% for i = 1:length(datas_zcom_loop)-1
for i = length(datas_zcom_loop)-1
    clf
    
    subplot(1,9,[1 2 3 4]); hold on; grid on; box on;
    plot3(times_zcom_loop(1:i),hss_zcom_loop(1:i), datas_zcom_loop(1:i)+0.005,'r','LineWidth',lw)
    mesh(times_zcom,hss_zcom,datas_zcom)
    plot3(times_zcom_loop(i),hss_zcom_loop(i), datas_zcom_loop(i)+0.005,'ro','MarkerSize',10,'MarkerFaceColor','r');
    ylabel('h (m)','interpreter','latex')
    xlabel('t (s)','interpreter','latex')
    zlabel('$z_{CoM}^d$ (m)','interpreter','latex')
    set(gca,'FontSize',fs)
    xlim([0,max(max(times_zcom_loop))])
    zlim([0.65 0.9])
    view([2 -3 1])
%     ax=gca;
%     set(ax,'SortMethod','childorder'); 

    subplot(1,9,[6 7 8 9]); hold on; grid on; box on;
    if times_zcom(idx,i) > t0 && times_zcom(idx,i) < tf
        try
            plot3(times_fz_loop(1:cnt_fz),hss_fz_loop(1:cnt_fz), datas_fz_loop(1:cnt_fz),'r','LineWidth',lw)
            plot3(times_fz_loop(cnt_fz),hss_fz_loop(cnt_fz), datas_fz_loop(cnt_fz)+0.005,'ro','MarkerSize',10,'MarkerFaceColor','r');
            cnt_fz = cnt_fz + 1;
        catch
            plot3(times_fz_loop(1:end),hss_fz_loop(1:end-1), datas_fz_loop(1:end),'r','LineWidth',lw)
        end
    elseif times_zcom(idx,i) > tf
        plot3(times_fz_loop(1:end),hss_fz_loop(1:end-1), datas_fz_loop(1:end),'r','LineWidth',lw)
    end
    m = mesh(times_fz,hss_fz,datas_fz);
%     m.FaceAlpha = 0.5;
    ylabel('h (m)','interpreter','latex')
    xlabel('t (s)','interpreter','latex')
    zlabel('$F_z^d$ (N)','interpreter','latex')
    set(gca,'FontSize',fs)
    % title('F_{Z}^d(t,h)','Interpreter','tex')
    xlim([0,max(max(times_zcom_loop))])
    zlim([0 2500])
    view([2 -3 1])
    
    % REPORT
    drawnow
    f.Position=[100 100 1400 700]; 
    f.Color = 'w';
    
    % VIDEO
%     drawnow
%     f.Position=[100 100 1800 700]; 
%     f.Color = 'w';
%     frame = getframe(gcf);
%     writeVideo(v,frame)
end
% close(v)
