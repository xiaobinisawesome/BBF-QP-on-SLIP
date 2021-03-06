%% generate a P2 orbit. 
test = backSteppingWalking;
test.genDesiredbehaviorSlopeDeg(30); 
test.LIP.gaitType = 'P2';
test.LIP.whichGain = 'deadbeat';
test.LIP.initializeLIP();
test.LIP.setDesiredVelocity(0); 
test.LIP.setP2orbitUleft(-0.2);
test.swingHeight = 0.35;
test.setDuration(16); 
test.simBackStepping;
test.plot;
test.plotLIP;
test.animate;
%%
log = struct(test);
timenow = datestr(now,30);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\RoughTerrainSLIP\simDataRev\';
save([folderName, 'Slope\', '50cmSpeed-P1', timenow,'.mat'], 'log');

%% test OUTPUT QP 
test = backSteppingWalking;
test.genDesiredbehaviorArbitrary; 
test.LIP.gaitType = 'P1';
test.LIP.whichGain = 'deadbeat';
test.LIP.initializeLIP();
test.LIP.setDesiredVelocity(0.5); 
test.swingHeight = 0.35;
test.setDuration(7); 
test.simBackStepping;
test.plot;
test.plotLIP;
test.animate;
%%
log = struct(test);
timenow = datestr(now,30);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\RoughTerrainSLIP\simDataRev\';
save([folderName, 'Slope\', '50cmSpeed-P1', timenow,'.mat'], 'log');

%% generate rough data 
test = backSteppingWalking;
test.genDesiredbehaviorArbitrary; 
test.setVelocity(0.5); 
test.swingHeight = 0.35;
test.setDuration(16); 
test.simBackStepping;
test.plot;
test.animate;

%%
log = struct(test);
timenow = datestr(now,30);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'rough\', '50cmSpeed-10delta', timenow,'.mat'], 'log');

%% parameters

test = backSteppingWalking;
test.genDesiredbehaviorArbitrary; 
%%
test.LIP.whichGain = 'deadbeat';
test.LIP.initializeLIP();
test.K = 8000; 
test.D = 100;
test.Ksim = 8000; %% 8000
test.Dsim = 100; %100; 
test.setVelocity(0.5); 
test.setDuration(16); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);
test.plotLIP;

%%
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\RoughTerrainSLIP\simDataRev\';
save([folderName, 'Rough\','Ksim8800-Dsim110', '50cmSpeed', timenow,'.mat'], 'log');


%% flat 

test = backSteppingWalking;
test.genDesiredbehaviorFlat; 
test.LIP.whichGain = 'deadbeat';
test.LIP.initializeLIP();
test.setVelocity(0.5); 
test.setDuration(10); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);
test.plotLIP;

%%%
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'flat\', '50cmSpeed', timenow,'.mat'], 'log');

%% Slope 
test = backSteppingWalking;
test.LIP.whichGain = 'deadbeat';
test.LIP.initializeLIP();
test.genDesiredbehaviorSlopeDeg(30); 
test.setVelocity(0.5); 
test.setDuration(6); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\RoughTerrainSLIP\simDataRev\';
save([folderName, 'slope\', '30deg-50cmSpeed', timenow,'.mat'], 'log');

%% SlopeUp and Down
test = backSteppingWalking;
test.genDesiredbehaviorSlopeDeg([-28,-28]); 
test.setVelocity(0.5); 
test.setDuration(12); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'slope\', 'n30deg-n30deg-50cmSpeed', timenow,'.mat'], 'log');

test.plotLIP;

degs = [-30, -20, -10, 10, 20, 30];
for i = 1:6
test = backSteppingWalking;
test.genDesiredbehaviorSlopeDeg([degs(i),degs(i)]); 
test.setVelocity(0.5); 
test.setDuration(12); 
test.simBackStepping;
%test.plot;
%test.animate;
timenow = datestr(now,30);
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'slope\deadbeat\', num2str(degs(i)), 'deg-50cmSpeed', timenow, '.mat'], 'log');
end
test.plotLIP;

%% stair
test = backSteppingWalking;
test.genDesiredbehaviorUpStairs(); 
test.setVelocity(0.5); 
test.setDuration(10); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);

log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'stairs\', '50cmSpeed', timenow,'.mat'], 'log');


%% up  stair
test = backSteppingWalking;
test.genDesiredbehaviorUpUpStairs(); 
test.setVelocity(0.4); 
test.setDuration(14); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);

log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'stairs\', 'upup50cmSpeed', timenow,'.mat'], 'log');

%% sinu
test = backSteppingWalking;
test.genDesiredbehaviorSinu(); 
test.setVelocity(0.5); 
test.setDuration(16); 
test.simBackStepping;
test.plot;
test.animate;
timenow = datestr(now,30);

log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'sinu\', 'new50cmSpeed', timenow,'.mat'], 'log');


%% gen flat velocity

for vel = 0.1:0.1:1.5
test = backSteppingWalking;
test.genDesiredbehaviorFlat; 
test.setVelocity(vel); 
test.setDuration(10); 
test.swingHeight = 0.2;
test.simBackStepping;
%test.plot;
%test.animate;
%test.plotLIP;

%%%
log = struct(test);
folderName = 'X:\OneDrive - California Institute of Technology\Pub\ICRA2021\simData\';
save([folderName, 'flat\Deadbeat\', 'vel-',num2str(vel*100), 'cmSpeed','.mat'], 'log');
end
 
