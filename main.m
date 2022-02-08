close all;
clear all; 
clc;

%% Adding paths
addpath('beziers')      % fitted bezier splines
addpath('data')         % human data
addpath('functions')    % helper functions
addpath('HLIP')         % LIP class
addpath('results')
addpath('Third')

%% flat
test = backSteppingWalking('human');
% test.genDesiredbehaviorFlat;
test.genDesiredbehaviorUnexpDownstep(0.025);
% test.genDesiredbehaviorExpDownstep(0.01);

% add LIP model
test.LIP.initializeLIP();
% set the forward velocity
test.useIncreasingVelocity   = true;
test.useIncreasingDeviation  = true;
test.useDecreasingRelaxation = true;
test.useSwingFootDetection   = true;

test.stepsToTrueDesired = 3;
test.setVelocityFromBezier(); 
% set vertical position from splines

xData = 0:0.01:1;
zData = cos(xData);
fun = @(x) costFcnBezier(x,xData,zData);
bv_0 = [0 0 0 0 0];
bv_zcom = fmincon(fun,bv_0);

test.useHumanZ = true;
test.useHumanF = true;
test.c_relax_SSP = 0.30;
test.c_relax_DSP = 0.30;

% run
test.setDuration(10); 
test.simBackStepping;
% plot
test.plot;
test.animate;
timenow = datestr(now,30);
test.plotLIP;

%%
logData = false;
if logData
    log = struct(test);
    folderName = 'results/';
    save([folderName,'flat', timenow,'.mat'], 'log');
end
