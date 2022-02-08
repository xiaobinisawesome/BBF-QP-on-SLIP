clear; 
% close all;

load('data/human/processedIKID.mat')
data = dataIKID;

fs = 13;

idxX = getIdxCoM(data,'center_of_mass_X');
idxY = getIdxCoM(data,'center_of_mass_Y');
idxZ = getIdxCoM(data,'center_of_mass_Z');

%% get the approximated mean of all the trials for each experiment
idxs.exp00  = [1 3;  1 4;  1 3;  1 4;  3 5;  1 3;  2 6;  1 4];
idxs.exp25  = [1 3;  1 3;  1 4;  1 3;  1 4;  1 4;  1 3;  1 3];
idxs.exp50  = [3 6;  1 3;  2 4;  2 4;  2 5;  2 4;  2 5;  4 6];
idxs.exp75  = [2 4;  1 4;  2 4;  1 3;  1 3;  2 5;  3 5;  1 3];
idxs.exp100 = [1 3;  2 4;  3 6;  2 5;  1 3;  1 3;  2 4;  2 4];

idxs.unexp25  = [2 6;  2 6;  1 4;  1 4;  1 4;  2 6;  1 4;  1 4];
idxs.unexp50  = [1 3;  1 3;  1 3;  2 4;  1 3;  2 5;  1 5;  1 3];
idxs.unexp75  = [1 5;  2 4;  1 4;  1 4;  1 6;  2 7;  2 4;  1 4];
idxs.unexp100 = [2 5;  1 3;  1 4;  2 5;  1 4;  1 4;  1 4;  1 3];


clearvars x y z t dx dy dz ddx ddy ddz
fields = fieldnames(data);
for fn = 1:length(fields)
    numFields = length(data.(fields{fn}));
    
%     figure; hold on; grid on;
    for i = 1:numFields
        zeroCrossingT = data.(fields{fn}){i}.IK.zeroCrossingT;
        [pkv,pki] = findpeaks(data.(fields{fn}){i}.IK.valuesCoM(:,idxY),...
                              'MinPeakHeight',0.5);
        
        idx0 = idxs.(fields{fn})(i,1);
        idxf = idxs.(fields{fn})(i,2);
        
        t0 = pki(idx0);
        tf = pki(idxf);

%         clf
%         hold on; grid on;
%         plot(data.(fields{fn}){i}.IK.valuesCoM(:,idxY))
%         plot(pki,0.87*ones(size(pki)),'ro');

        x{i} = data.(fields{fn}){i}.IK.valuesCoM(t0:tf,idxX)-data.(fields{fn}){i}.IK.valuesCoM(t0,idxX);
        y{i} = data.(fields{fn}){i}.IK.valuesCoM(t0:tf,idxY);%-data.(fields{fn}){i}.IK.valuesCoM(t0,idxY);
        z{i} = data.(fields{fn}){i}.IK.valuesCoM(t0:tf,idxZ);%-data.(fields{fn}){i}.IK.valuesCoM(t0,idxZ);
        t{i} = data.(fields{fn}){i}.IK.values(t0:tf,1)-data.(fields{fn}){i}.IK.values(t0,1);
        
        dx{i} = data.(fields{fn}){i}.IK.dvaluesCoM(t0:tf,idxX);
        dy{i} = data.(fields{fn}){i}.IK.dvaluesCoM(t0:tf,idxY);
        dz{i} = data.(fields{fn}){i}.IK.dvaluesCoM(t0:tf,idxZ);
        
        ddx{i} = data.(fields{fn}){i}.IK.ddvaluesCoM(t0:tf,idxX);
        ddy{i} = data.(fields{fn}){i}.IK.ddvaluesCoM(t0:tf,idxY);
        ddz{i} = data.(fields{fn}){i}.IK.ddvaluesCoM(t0:tf,idxZ);
%         plot(y{i})
    end
%     title(fields{fn})

    
    [xinterp,yinterp,zinterp,tinterp] = interpolateWide(x,y,z,t,fn);
    meanCoM.(fields{fn}).x = xinterp;
    meanCoM.(fields{fn}).y = yinterp;
    meanCoM.(fields{fn}).z = zinterp;
    meanCoM.(fields{fn}).t = tinterp;
    
    [dxinterp,dyinterp,dzinterp,~] = interpolateWide(dx,dy,dz,t,fn);
    meanCoM.(fields{fn}).dx = dxinterp;
    meanCoM.(fields{fn}).dy = dyinterp;
    meanCoM.(fields{fn}).dz = dzinterp;
    
    [ddxinterp,ddyinterp,ddzinterp,~] = interpolateWide(ddx,ddy,ddz,t,fn);
    meanCoM.(fields{fn}).ddx = ddxinterp;
    meanCoM.(fields{fn}).ddy = ddyinterp;
    meanCoM.(fields{fn}).ddz = ddzinterp;
end

% interpolate time dependent dimensions and x-y plane dimension
order = 7;
for fn = 1:length(fields)
    meanCoM.(fields{fn}).ptx = polyfit(meanCoM.(fields{fn}).t,...
                                       meanCoM.(fields{fn}).x,order);
    meanCoM.(fields{fn}).pty = polyfit(meanCoM.(fields{fn}).t,...
                                       meanCoM.(fields{fn}).y,order);
                                   
    meanCoM.(fields{fn}).ptdx = polyfit(meanCoM.(fields{fn}).t,...
                                        meanCoM.(fields{fn}).dx,order);
    meanCoM.(fields{fn}).ptdy = polyfit(meanCoM.(fields{fn}).t,...
                                        meanCoM.(fields{fn}).dy,order);
                                   
    meanCoM.(fields{fn}).ptddx = polyfit(meanCoM.(fields{fn}).t,...
                                         meanCoM.(fields{fn}).ddx,order);
    meanCoM.(fields{fn}).ptddy = polyfit(meanCoM.(fields{fn}).t,...
                                         meanCoM.(fields{fn}).ddy,order);
                                   
    meanCoM.(fields{fn}).pxy = polyfit(meanCoM.(fields{fn}).x,...
                                       meanCoM.(fields{fn}).y,order);
                                   
    meanCoM.(fields{fn}).pxdx = polyfit(meanCoM.(fields{fn}).x,...
                                        meanCoM.(fields{fn}).dx,order);
    meanCoM.(fields{fn}).pxdy = polyfit(meanCoM.(fields{fn}).x,...
                                        meanCoM.(fields{fn}).dy,order);

    meanCoM.(fields{fn}).pxddx = polyfit(meanCoM.(fields{fn}).x,...
                                         meanCoM.(fields{fn}).ddx,order);
    meanCoM.(fields{fn}).pxddy = polyfit(meanCoM.(fields{fn}).x,...
                                         meanCoM.(fields{fn}).ddy,order);
end

figure; 
subplot(2,2,1); hold on; grid on;
for fn = [1 2 3 4 5]
    plot(meanCoM.(fields{fn}).x,meanCoM.(fields{fn}).y)
end
xlabel('xcom')
title('expected')
subplot(2,2,2); hold on; grid on;
for fn = [1 6 7 8 9]
    plot(meanCoM.(fields{fn}).x,meanCoM.(fields{fn}).y)
end
xlabel('xcom')
title('unexpected')

subplot(2,2,3); hold on; grid on;
for fn = [1 2 3 4 5]
    plot(meanCoM.(fields{fn}).t,meanCoM.(fields{fn}).y)
end
xlabel('time')
title('expected')

subplot(2,2,4); hold on; grid on;
for fn = [1 6 7 8 9]
    plot(meanCoM.(fields{fn}).t,meanCoM.(fields{fn}).y)
end
xlabel('time')
title('unexpected')

linkaxes

%% XCOM FITTING
% assume function: y = B + A*cos(w*x + phi)
% B0 = 0.85; A0 = 0.1; w0 = 2*pi; phi0 = 0;
% yMean = [meanCoM.exp00.y(1:end/2)];
% xMean = [meanCoM.exp00.x(1:end/2)];
% xysol = fminsearch(@(t) sum( (yMean(:) - ( t(1)+t(2)*cos(t(3)*xMean(:)+t(4))) ).^2 ),...
%     [B0 A0 w0 phi0]);
% % xysol(4) = 0;
% xMean = [meanCoM.exp00.x];
% xycosine = xysol(1) + xysol(2)*(cos(xysol(3).*xMean + xysol(4)));

B0 = 0.85; A0 = 0.1; w0 = 2*pi; phi0 = 0;
yMean = [meanCoM.exp00.y(1:end)];
xMean = [meanCoM.exp00.x(1:end)];
xysol = fminsearch(@(t) sum( (yMean(:) - ( t(1)+t(2)*cos(t(3)*xMean(:))) ).^2 ),...
    [B0 A0 w0]);
xEval = linspace(0,1.60,length(meanCoM.exp00.x));
xycosine = xysol(1) + xysol(2)*(cos(xysol(3).*xEval));

meanCoM.exp00.y = xycosine';


% now fit 7th order polynomial to downsteps
fun7 = @(p) sum( (yMean(:) - (p(1)*xMean(:).^7 + p(2)*xMean(:).^6 + ...
    p(3)*xMean(:).^5 + p(4)*xMean(:).^4 + p(5)*xMean(:).^3 + ...
    p(6)*xMean(:).^2 + p(7)*xMean(:) + p(8))).^2);
nlc7 = @(p) ([p(end) - (xysol(1)+abs(xysol(2))); p(end-1)]);
for fn = 1:length(fields)
    xMean = meanCoM.(fields{fn}).x;
    yMean = meanCoM.(fields{fn}).y;
    xm = max(xMean);
    fun7 = @(p) sum( (yMean(:) - (p(1)*xMean(:).^7 + p(2)*xMean(:).^6 + ...
        p(3)*xMean(:).^5 + p(4)*xMean(:).^4 + p(5)*xMean(:).^3 + ...
        p(6)*xMean(:).^2 + p(7)*xMean(:) + p(8))).^2);
    nlc7 = @(p) deal([2*p(6)],...
        [p(8) - (xysol(1)+abs(xysol(2))); 
         p(1)*xm^7+p(2)*xm^6+p(3)*xm^5+p(4)*xm^4+p(5)*xm^3+p(6)*xm^2+p(7)*xm+p(8) - (xysol(1)+abs(xysol(2)));
         p(7);
         7*p(1)*xm^6+6*p(2)*xm^5+5*p(3)*xm^4+4*p(4)*xm^3+3*p(5)*xm^2+2*p(6)*xm+p(7)]);
    p0 = meanCoM.(fields{fn}).pxy;
    meanCoM.(fields{fn}).pxy_nlc = fmincon(fun7,p0,...
        [],[],[],[],[],[],nlc7);
end


%% TIME FITTING
% assume function: y = B + A*cos(w*x + phi)
% B0 = 0.85; A0 = 0.1; w0 = 2*pi; phi0 = 0;
% yMean = [meanCoM.exp00.y(1:end/2)];
% tMean = [meanCoM.exp00.t(1:end/2)];
% tysol = fminsearch(@(t) sum( (yMean(:) - ( t(1)+t(2)*cos(t(3)*tMean(:)+t(4))) ).^2 ),...
%     [B0 A0 w0 phi0]);
% tysol(4) = 0;
% tMean = [meanCoM.exp00.t];
% tycosine = tysol(1) + tysol(2)*(cos(tysol(3).*tMean + tysol(4)));

B0 = 0.85; A0 = 0.1; w0 = 4*pi; phi0 = 0;
yMean = [meanCoM.exp00.y(1:end)];
tMean = [meanCoM.exp00.t(1:end)];
tysol = fminsearch(@(t) sum( (yMean(:) - ( t(1)+t(2)*cos(t(3)*tMean(:)) ) ).^2 ),...
    [B0 A0 w0]);
tEval = meanCoM.exp00.t;
tycosine = tysol(1) + tysol(2)*(cos(tysol(3).*tEval));


% now fit 7th order polynomial to downsteps
fun7 = @(p) sum( (yMean(:) - (p(1)*tMean(:).^7 + p(2)*tMean(:).^6 + ...
    p(3)*tMean(:).^5 + p(4)*tMean(:).^4 + p(5)*tMean(:).^3 + ...
    p(6)*tMean(:).^2 + p(7)*tMean(:) + p(8))).^2);
nlc7 = @(p) ([p(end) - (tysol(1)+abs(tysol(2))); p(end-1)]);
for fn = 1:length(fields)
    tMean = meanCoM.(fields{fn}).t;
    yMean = meanCoM.(fields{fn}).y;
    xm = max(tMean);
    fun7 = @(p) sum( (yMean(:) - (p(1)*tMean(:).^7 + p(2)*tMean(:).^6 + ...
        p(3)*tMean(:).^5 + p(4)*tMean(:).^4 + p(5)*tMean(:).^3 + ...
        p(6)*tMean(:).^2 + p(7)*tMean(:) + p(8))).^2);
    nlc7 = @(p) deal([2*p(6)],...
        [p(8) - (tysol(1)+abs(tysol(2))); 
         p(1)*xm^7+p(2)*xm^6+p(3)*xm^5+p(4)*xm^4+p(5)*xm^3+p(6)*xm^2+p(7)*xm+p(8) - (tysol(1)+abs(tysol(2)));
         p(7);
         7*p(1)*xm^6+6*p(2)*xm^5+5*p(3)*xm^4+4*p(4)*xm^3+3*p(5)*xm^2+2*p(6)*xm+p(7)]);
    p0 = meanCoM.(fields{fn}).pxy;
    meanCoM.(fields{fn}).pty_nlc = fmincon(fun7,p0,...
        [],[],[],[],[],[],nlc7);
end



%% Plotting
figure;
subplot(2,2,1); hold on; grid on;
xMean = meanCoM.exp00.x; %[meanCoM.exp00.x(18:end); meanCoM.exp00.x(1:17)+meanCoM.exp00.x(end)];
plot(xMean,xycosine,'LineWidth',2)
for fn = [2 3 4 5]
    xMean = meanCoM.(fields{fn}).x;
    yPoly = polyval(meanCoM.(fields{fn}).pxy_nlc,xMean);
    plot(xMean,yPoly,'LineWidth',2)
end
legend('exp00','exp25','exp50','exp75','exp100')
xlabel('x CoM position [m]')
ylabel('y CoM position [m]')
title('Expected down-step')
set(gca,'FontSize',15)

subplot(2,2,2); hold on; grid on;
xMean = meanCoM.exp00.x;
plot(xMean,xycosine,'LineWidth',2)
for fn = [6 7 8 9]
    xMean = meanCoM.(fields{fn}).x;
    yPoly = polyval(meanCoM.(fields{fn}).pxy_nlc,xMean);
    plot(xMean,yPoly,'LineWidth',2)
end
legend('exp00','unexp25','unexp50','unexp75','unexp100')
xlabel('x CoM position [m]')
ylabel('y CoM position [m]')
title('Unexpected down-step')
set(gca,'FontSize',15)




subplot(2,2,3); hold on; grid on;
tMean = meanCoM.exp00.t; %[meanCoM.exp00.x(18:end); meanCoM.exp00.x(1:17)+meanCoM.exp00.x(end)];
plot(tMean,tycosine,'LineWidth',2)
for fn = [2 3 4 5]
    tMean = meanCoM.(fields{fn}).t;
    yPoly = polyval(meanCoM.(fields{fn}).pty_nlc,tMean);
    plot(tMean,yPoly,'LineWidth',2)
end
legend('exp00','exp25','exp50','exp75','exp100')
xlabel('time [s]')
ylabel('y CoM position [m]')
title('Expected down-step')
set(gca,'FontSize',15)

subplot(2,2,4); hold on; grid on;
tMean = meanCoM.exp00.t;
plot(tMean,tycosine,'LineWidth',2)
for fn = [6 7 8 9]
    tMean = meanCoM.(fields{fn}).t;
    yPoly = polyval(meanCoM.(fields{fn}).pty_nlc,tMean);
    plot(tMean,yPoly,'LineWidth',2)
end
legend('exp00','unexp25','unexp50','unexp75','unexp100')
xlabel('time [s]')
ylabel('y CoM position [m]')
title('Unexpected down-step')
set(gca,'FontSize',15)

linkaxes




%%% COMPARISON WITH RAW DATA
figure; 
subplot(1,2,1); hold on; grid on;
subplot(1,2,2); hold on; grid on;
for i = 1:8
        [pkv,pki] = findpeaks(data.('exp00'){i}.IK.valuesCoM(:,idxY),...
                              'MinPeakHeight',0.5);
        idx0 = idxs.('exp00')(i,1);
        idxf = idxs.('exp00')(i,2);
        t0 = pki(idx0);
        tf = pki(idxf);

        x = data.('exp00'){i}.IK.valuesCoM(t0:tf,idxX)-data.('exp00'){i}.IK.valuesCoM(t0,idxX);
        y = data.('exp00'){i}.IK.valuesCoM(t0:tf,idxY);
        t = data.('exp00'){i}.IK.values(t0:tf,1)-data.('exp00'){i}.IK.values(t0,1);
        subplot(1,2,1);
        plot(x,y);
        subplot(1,2,2);
        plot(t,y);
end
subplot(1,2,1);
plot(meanCoM.exp00.x,meanCoM.exp00.y,'b','LineWidth',2)
plot(meanCoM.exp00.x,xycosine,'r','LineWidth',2)
xlabel('xcom')

subplot(1,2,2);
plot(meanCoM.exp00.t,meanCoM.exp00.y,'b','LineWidth',2)
plot(meanCoM.exp00.t,tycosine,'r','LineWidth',2)
xlabel('time')
legend('','','','','','','','','mean','poly')

%% EVENTS
xcomEvents.exp00.raw  = [0 0.3054 0.5127 0.8186 1.116 1.347 1.593];
xcomEvents.exp25.raw  = [0 0.2981 0.554  0.9025 1.168 1.377 1.599];
xcomEvents.exp50.raw  = [0 0.352  0.587  0.9497 1.218 1.379 1.653];
xcomEvents.exp75.raw  = [0 0.4143 0.5958 0.9762 1.203 1.363 1.616];
xcomEvents.exp100.raw = [0 0.4497 0.624  1.002  1.226 1.360 1.636];
xcomEvents.unexp25.raw  = [0 0.3086 0.5377 0.9139 1.226 1.379 1.605];
xcomEvents.unexp50.raw  = [0 0.3676 0.5703 0.9431 1.224 1.381 1.652];
xcomEvents.unexp75.raw  = [0 0.4415 0.6308 0.9738 1.251 1.421 1.705];
xcomEvents.unexp100.raw = [0 0.5095 0.6733 1.006  1.269 1.428 1.75];

timeEvents = struct;
for fn = 1:length(fields)
    xcom = meanCoM.(fields{fn}).x;
    time = meanCoM.(fields{fn}).t;
    idxs = [];
    for ii = 1:length(xcomEvents.(fields{fn}).raw)
        [~,idx] = min(abs(xcom - xcomEvents.(fields{fn}).raw(ii)));
        idxs = [idxs idx];
    end
    times = time(idxs);
    timeEvents.(fields{fn}).raw = time(idxs);
    timeEvents.(fields{fn}).rawIdx = idxs;
    timeEvents.(fields{fn}).phase1.timeMax = times(3) - times(1);
    timeEvents.(fields{fn}).phase1.timeMax_SSP = times(2) - times(1);
    timeEvents.(fields{fn}).phase2.timeMax = times(6) - times(3);
    timeEvents.(fields{fn}).phase2.timeMax_SSP = times(5) - times(3);
    timeEvents.(fields{fn}).phase3.timeMax = times(7) - times(6);
    timeEvents.(fields{fn}).phase3.timeMax_SSP = times(7) - times(6);
    timeEvents.(fields{fn}).phase1.timeMaxIdx = idxs(3);
    timeEvents.(fields{fn}).phase2.timeMaxIdx = idxs(6);
    timeEvents.(fields{fn}).phase3.timeMaxIdx = idxs(7);
    
    xcoms = xcomEvents.(fields{fn}).raw;
    xcomEvents.(fields{fn}).rawIdx = idxs;
    xcomEvents.(fields{fn}).phase1.xcomMax = xcoms(3) - xcoms(1);
    xcomEvents.(fields{fn}).phase2.xcomMax = xcoms(6) - xcoms(3);
    xcomEvents.(fields{fn}).phase3.xcomMax = xcoms(7) - xcoms(6);
    xcomEvents.(fields{fn}).phase1.xcomMaxIdx = idxs(3);
    xcomEvents.(fields{fn}).phase2.xcomMaxIdx = idxs(6);
    xcomEvents.(fields{fn}).phase3.xcomMaxIdx = idxs(7);
end

save('xcomEvents.mat','xcomEvents')
save('timeEvents.mat','timeEvents')
humanDataCoM = meanCoM;
save('humanDataCoM.mat','humanDataCoM')



%% helper functions
function idx = getIdx(data,name)
    idx = find(contains(data.exp00{1}.IK.headers,name));
end

function idx = getIdxMean(data,name)
    idx = find(contains(data.exp00{1}.IK.headersMean,name));
end

function idx = getIdxSD(data,name)
    idx = find(contains(data.exp00{1}.IK.headersSD,name));
end

function idx = getIdxCoM(data,name)
    idx = find(contains(data.exp00{1}.IK.headersCoM,name));
end

function idx = getIdxDyn(data,name)
    idx = find(contains(data.exp00{1}.ID.headersDyn,name));
end

function [xinterp,yinterp,zinterp,tinterp] = interpolateWide(x,y,z,t,run)
    % get index of run that contains takes the most time
    idx = 0;
    minVal = 10;
    for i = 1:length(t)
        if max(t{i}) < minVal
            minVal = max(t{i});
            idx = i;
        end
    end
    tinterp = t{idx};
    
    % interpolate x
    maxdiff = inf;
    considered = 1:length(x);
    while maxdiff > 0.05
        xInterp = [];
        for i = considered
            xInterp = [xInterp interp1(t{i},x{i},tinterp)];
        end
        xinterp = mean(xInterp,2,'omitnan');
        diff = abs(xInterp-xinterp);
        [valRow,~] = max(diff,[],1);
        [maxdiff,idxCol] = max(valRow);
        
        considered(idxCol) = [];
    end
    
    % interpolate y
    maxdiff = inf;
    considered = 1:length(y);
    while maxdiff > 0.05
        yInterp = [];
        for i = considered
            yInterp = [yInterp interp1(t{i},y{i},tinterp)];
        end
        yinterp = mean(yInterp,2,'omitnan');
        diff = abs(yInterp-yinterp);
        [valRow,~] = max(diff);
        [maxdiff,idxCol] = max(valRow);
        
        considered(idxCol) = [];
    end
    
    % interpolate z
    maxdiff = inf;
    considered = 1:length(z);
    while maxdiff > 0.05
        zInterp = [];
        for i = considered
            zInterp = [zInterp interp1(t{i},z{i},tinterp)];
        end
        zinterp = mean(zInterp,2,'omitnan');
        diff = abs(zInterp-zinterp);
        [valRow,~] = max(diff);
        [maxdiff,idxCol] = max(valRow);
        
        considered(idxCol) = [];
    end        
end