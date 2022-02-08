clear all; 
close all;
addpath('functions/')

%% Read the outputs
% fid = fopen('vx_new/qp_y_walk_log_SLIP.txt','r');
% qp_y = [];
% cntArray = 1; cntTotal = 1;
% line = fgets(fid);
% while ischar(line)
%     if isnan(str2double(line))
%         cntTotal = cntTotal+1;
%         cntArray = 1;
%     else
%         qp_y(cntArray,cntTotal) = str2double(line);
%         cntArray = cntArray+1;
%     end
%     line = fgets(fid);
% end
% 
% save('qp_y.mat','qp_y')
load('qp_y.mat')
 
%% Plot the outputs
outputs = {'deltaRoll','deltaPitch','StanceHipYaw','zcom','zsw','xsw','ysw',...
           'swHipYaw','deltaSwingFoot','xcom'};

ylim_lb = [-0.2 -0.2 -0.2 0.7 -0.02 -0.25 -0.5 -0.25 -0.25 -0.15];
ylim_ub = [0.2   0.2  0.2 0.8  0.15  0.25  0.5  0.25  0.25  0.15];
figure;
for i = 1:10
    subplot(2,5,i); hold on; grid on;
    plot(qp_y(4+i,:),'r')
    plot(qp_y(24+i,:),'b--')
    ylim([ylim_lb(i) ylim_ub(i)])
    title(outputs{i})
end
sgtitle('POSITION BASED RD2 OUTPUTS')

figure;
for i = 1:10
    subplot(2,5,i); hold on; grid on;
    plot(qp_y(14+i,:),'r')
    plot(qp_y(34+i,:),'b--')
    title(outputs{i})
end
sgtitle('VELOCITY BASED RD2 OUTPUTS')

figure;
for i = 1:10
    subplot(2,5,i); hold on; grid on;
    plot(qp_y(44+i,:),'b--')
    title(outputs{i})
end
sgtitle('ACCELERATION BASED RD2 OUTPUTS')

%% Preprinting
fid = fopen('/home/none/cassie_ws_2/src/cassie_controllers/include/cassie_controllers/generated/getDebugBeziers.hpp','w');
% fid = fopen('/home/none/trash.hpp','w');

fprintf(fid,'#include <cassie_common_toolbox/bezier_tools.hpp>\n');
fprintf(fid,'#ifndef DEBUG_BEZIERS_HPP\n');
fprintf(fid,'#define DEBUG_BEZIERS_HPP\n\n\n');
fprintf(fid,'void SPLINES_BEZIER::getDebugBeziers(){\n\n');

%% Divide into steps
time = qp_y(72,:);
ya = qp_y(5:15,:);
dya = qp_y(15:25,:);

yd = qp_y(25:35,:);
dyd = qp_y(35:45,:);
ddyd = qp_y(45:55,:);

[stepT,stepIdx] = findpeaks(time);


phases = 1:20;
idx1 = 1;
for i = 1:length(phases)
    phase = strcat('SSP',num2str(phases(i)));
    
    idx0 = idx1+3;
    idx1 = stepIdx(i)-1;
    
    time_phase = time(idx0:idx1) - time(idx0);
    time_norm = (time(idx0:idx1) - time(idx0))/...
                (time(idx1) - time(idx0));
    time_norm = linspace(0,1,length(time_norm));
    
    yaStep = ya(:,idx0:idx1);
    dyaStep = dya(:,idx0:idx1);
    
    ydStep = yd(:,idx0:idx1);
    dydStep = dyd(:,idx0:idx1);
    ddydStep = ddyd(:,idx0:idx1);
    
    %% pos
    zcom = ydStep(4,:);
    dzcom = dydStep(4,:);
    ddzcom = ddydStep(4,:);
    
    xcom = yaStep(10,:);
    dxcom = dydStep(10,:);
    dxcom = movmean(dxcom,25);
    ddxcom = ddydStep(10,:);
    
    zsw = ydStep(5,:);
    dzsw = dydStep(5,:);
    ddzsw = ddydStep(5,:);
    
    xsw = ydStep(6,:);
    dxsw = dydStep(6,:);
    ddxsw = ddydStep(6,:);
       
    dataToSpline = {zcom,xsw,xcom,...
                    dzcom,dxsw,dxcom,...
                    ddzcom,ddxsw,ddxcom};
    stringsToSpline = {'zcom','xsw','xcom',...
                       'dzcom','dxsw','dxcom',...
                       'ddzcom','ddxsw','ddxcom'};
        
    figure
    for ii = 1:length(dataToSpline)
        xdata = time_norm;
        zdata = dataToSpline{ii};
        
        half = round(length(zdata));
        bv0 = [zdata(1) zdata(1) zdata(half) zdata(end) zdata(end)];
        fun = @(x) costFcn(x,xdata,zdata);
        bvf = fmincon(fun,bv0,[],[],[0 0 0 0 0],[0])
        subplot(3,3,ii); hold on; grid on;
        plot(xdata,zdata)
        plot(xdata,evalBezier(bvf,xdata))
        title(stringsToSpline{ii})
        
        name = strcat('bv_',stringsToSpline{ii},'_debug');
        printBezierDownstep(fid,bvf,name,i)
    end
    fprintf(fid,strcat('timeMax_debug.push_back(',num2str(time_phase(end)),');\n\n\n'));
end

fprintf(fid,'}\n');
fprintf(fid,'#endif');


%% FUNCTIONS
function c = costFcn(bv,tdata,zdata)
    z = zeros(size(tdata));
    for i = 1:length(tdata)
        z(i) = bezier2(bv,tdata(i));
    end
    c = rms(z-zdata);
end

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

function printBezierDownstep(fid,data,name,cnt)
    fprintf(fid,'Eigen::VectorXd %s%i(%i);\n',name,cnt,length(data));
    fprintf(fid,'%s%i << %f,\n',name,cnt,data(1));
    for i = 2:length(data)-1
        fprintf(fid,'%f,\n',data(i));
    end
    fprintf(fid,'%f;\n',data(end));
    fprintf(fid,'%s.push_back(%s%i);\n\n',name,name,cnt);
end