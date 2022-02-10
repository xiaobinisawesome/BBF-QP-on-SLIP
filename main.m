% close all;
clear all; 
clc;

%% Adding paths
addpath('data')         % human data, beziers, interpolated beziers
addpath('functions')    % helper functions
addpath('HLIP')         % LIP class
addpath('results')
addpath('Third')

%% flat
test = backSteppingWalking('human');
test.genDesiredbehaviorFlat;
test.genDesiredbehaviorUnexpDownstep(0.10);
test.expectedDownstep = true;

% add LIP model
test.LIP.initializeLIP();
test.maxStepsize = 1.0;
% set the forward velocity
test.useIncreasingVelocity   = true;
test.useIncreasingDeviation  = true;
test.useDecreasingRelaxation = true;
test.useSwingFootDetection   = true;
test.useTimeBased            = true;    % xcom phasing for zcom
test.useNewNominalSpline     = false;    % new spline 

test.stepsToTrueDesired = 3;
test.setVelocityFromBezier(); 

% set vertical position from splines
test.useHumanZ = true;
test.useHumanF = true;
test.c_relax_SSP = 0.25; %0.30
test.c_relax_DSP = 0.25; %0.30

% run
test.setDuration(7); 
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
