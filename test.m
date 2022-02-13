clear all; 
close all;
clc;

%% Testing
system = 'Cassie';
OUTPUT = backSteppingOutput(system);
          
% set time
zdesLog = []; dzdesLog = []; ddzdesLog = [];
FdesLog = []; dFdesLog = [];
for step = 1:10
    times = linspace(0,OUTPUT.TS+OUTPUT.TD);
    for i = 1:length(times)
        time = times(i);
        if time < OUTPUT.TS
            NcontactLegs = 1;
        else
            NcontactLegs = 2;
        end


        % update
        isDownstep = false;
        downstepStep = 0;
        knownDownstepHeight = 0;
        downstepHeightDetected = 0.0;
        stepCnt = step;

        OUTPUT = OUTPUT.update(isDownstep,downstepStep,knownDownstepHeight,...
                               downstepHeightDetected,NcontactLegs,stepCnt);

        % set
        stepTime = time;
        stepTimeVLO = 0.0;
        zsw2f = 0.1;
        OUTPUT = OUTPUT.setTimeAndZsw(stepTime,stepTimeVLO,zsw2f);

        % get (passing 0.0 is legacy)
        [zdes,dzdes,ddzdes] = OUTPUT.getDesiredZ(0.0);
        [Fdes,dFdes] = OUTPUT.getDesiredF(0.0);

        zdesLog = [zdesLog; zdes];
        dzdesLog = [dzdesLog; dzdes];
        ddzdesLog = [ddzdesLog; ddzdes];
        if length(Fdes) == 1
            Fdes = [Fdes; 0];
            dFdes = [dFdes; 0];
        end
        FdesLog = [FdesLog Fdes];
        dFdesLog = [dFdesLog dFdes];
    end
end

figure
subplot(2,3,1); hold on; grid on;
plot(zdesLog)
subplot(2,3,2); hold on; grid on;
plot(dzdesLog)
subplot(2,3,3); hold on; grid on;
plot(ddzdesLog)

subplot(2,3,4); hold on; grid on;
plot(FdesLog(1,:))
plot(FdesLog(2,:))
subplot(2,3,5); hold on; grid on;
plot(dFdesLog(1,:))
plot(dFdesLog(2,:))

