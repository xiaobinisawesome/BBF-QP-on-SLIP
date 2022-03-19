close all;
clear all; 
clc;

%% Adding paths
addpath('data')         % human data, beziers, interpolated beziers
addpath('functions')    % helper functions
addpath('HLIP')         % LIP class
addpath('results')
addpath('Third')

%% flat
system = 'Cassie';
exp = 'unexp100';

test = backSteppingWalking(system,exp);
% test.genDesiredbehaviorFlat;
test.genDesiredbehaviorUnexpDownstep(0.10);
test.expectedDownstep = false;

% add LIP model
test.LIP.initializeLIP();
test.maxStepsize = Inf; % exp: 0.80;
% set the forward velocity
test.useIncreasingVelocity   = true;
test.useIncreasingDeviation  = true;
test.useDecreasingRelaxation = true;
test.useSwingFootDetection   = true;
test.useTimeBased            = true;    % xcom phasing for zcom

test.stepsToTrueDesired = 5;
test.setVelocityFromBezier(); 

% set vertical position from splines
test.useHumanZ = true;
test.useHumanF = true;
test.c_relax_SSP = 0.35; %0.35
test.c_relax_DSP = 0.35; %0.35
test.c_relax_SSP_downstep = 0.35; %0.35
test.c_relax_DSP_downstep = 0.35; %0.35

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
    save([folderName,exp,system,'.mat'], 'log');
end
