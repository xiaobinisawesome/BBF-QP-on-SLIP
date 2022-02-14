clear all; close all; clc;

%% Load data
load('data/human/rawIKID.mat')
dataR = rawIKID;
load('data/human/processedIKID.mat')
dataP = dataIKID;

%% Begin
fields = fieldnames(dataR);

Lstep_struct = struct;
zsw_struct = struct;
for i = 1:9   
    Lstep = []; Tstep = [];
    zsw1 = []; zsw2 = []; zsw3 = []; zsw4 = [];
    dzsw1 = []; dzsw2 = []; dzsw3 = []; dzsw4 = [];
    
    zswt1 = []; zswt2 = []; zswt3 = []; zswt4 = [];
    dzswt1 = []; dzswt2 = []; dzswt3 = []; dzswt4 = [];
    time1 = []; time2 = []; time3 = []; time4 = [];
    
    exp = fields{i};
    for trial = 1:8     % number of trials
        %% Load the data
        dataPi = dataP.(exp){trial}.IK;
        dataRi = dataR.(exp){trial}.IK;
        isRightLegVLO = dataPi.rightLegVLO;

        if isRightLegVLO
            st_idx = getIdxCoM(dataPi,'toes_r_X');
            sw_idx = getIdxCoM(dataPi,'toes_l_X');
            Tst_idx = getIdxCoM(dataPi,'talus_r_Y');
            Tsw_idx = getIdxCoM(dataPi,'talus_l_Y');
        else
            st_idx = getIdxCoM(dataPi,'toes_l_X');
            sw_idx = getIdxCoM(dataPi,'toes_r_X'); 
            Tst_idx = getIdxCoM(dataPi,'talus_r_Y');
            Tsw_idx = getIdxCoM(dataPi,'talus_l_Y');
        end

        time = dataRi.valuesCoM(:,1);
        xst = dataRi.valuesCoM(:,st_idx);
        xsw = dataRi.valuesCoM(:,sw_idx);
        Txst = dataRi.valuesCoM(:,Tst_idx-1);
        Txsw = dataRi.valuesCoM(:,Tsw_idx-1);
        Tzst = dataRi.valuesCoM(:,Tst_idx);
        Tzsw = dataRi.valuesCoM(:,Tsw_idx);
        Txzst = dataRi.dvaluesCoM(:,Tst_idx-1);
        Txzsw = dataRi.dvaluesCoM(:,Tsw_idx-1);
        Tdzst = dataRi.dvaluesCoM(:,Tst_idx);
        Tdzsw = dataRi.dvaluesCoM(:,Tsw_idx);

%         figure; 
%         subplot(1,2,1); hold on; grid on;
%         plot(Tzst,'r')
%         plot(Tzsw,'b')
%         plot(Tdzst,'r--')
%         plot(Tdzsw,'b--')
%         xlabel('time [s]')
%         ylabel('zsw, dzsw [m/s,m/s^2]')
%         
%         subplot(1,2,2); hold on; grid on;
%         plot(Txst,Tzst,'r')
%         plot(Txsw,Tzsw,'b')
%         xlabel('xsw [m]')
%         ylabel('zsw [m]')
        
        %% Get static points
        % now find where for 22 idxs we have a variance lower than alpha
        xst_static = zeros(size(xst));
        xsw_static = zeros(size(xsw));
        range = 22;
        cnt = 1;
        while cnt <= length(xst)-range
            if var(xst(cnt:cnt+range)) < 0.0001
                xst_static(cnt) = 1;
                if cnt == length(xst)-range
                    xst_static(cnt:end) = 1;
                    break;
                end
            else
                xst_static(cnt) = 0;
            end
            if var(xsw(cnt:cnt+range)) < 0.0001
                xsw_static(cnt) = 1;
                if cnt == length(xst)-range
                    xsw_static(cnt:end) = 1;
                    break;
                end
            else
                xsw_static(cnt) = 0;
            end
            cnt = cnt + 1;
        end
        
%         plot(xst_static)
%         plot(xsw_static)
%         legend('st','sw','dst','dsw','st','sw')
        
%         figure; hold on; grid on;
%         plot(xst)
%         plot(xsw)
%         plot(xst_static)
%         plot(xsw_static)

        %% Get the positions of the feet in an alternating array
        step_array = [];
        stepTime_array = [];
        cnt = 1;
        while cnt < length(xsw_static)
            if xsw_static(cnt) == 1     % stance
                step_array = [step_array xsw(cnt)];
                cnt = cnt + find(xsw_static(cnt:end)==0,1,'first');
                stepTime_array = [stepTime_array time(cnt)];
            elseif xst_static(cnt) == 1
                step_array = [step_array xst(cnt)];
                cnt = cnt + find(xst_static(cnt:end)==0,1,'first');
                stepTime_array = [stepTime_array time(cnt)];
            else
                cnt = cnt+1;
            end
        end
        step_sizes = diff(step_array);
        stepTimes = diff(stepTime_array);
        Lstep = [Lstep; step_sizes(1:4)];
        Tstep = [Tstep; stepTimes(1:3)];
        
        %% Get the vertical swing foot trajectories
        zsw_norm_array = []; dzsw_norm_array = []; cnt_norm_array = []; % normalized 0 1
        zsw_time_array = {}; dzsw_time_array = {}; time_array = {}; % with time
        cnt = 1;
        while cnt < length(xsw_static)
            if xst_static(cnt) == 1     % swing
                cntEnd = cnt + find(xst_static(cnt:end)==0,1,'first');
                if isempty(cntEnd)
                    cntEnd = length(xst_static);
                    Talus_zsw = Tzsw(cnt:cntEnd);
                    Talus_dzsw = Tdzsw(cnt:cntEnd);
                    tsw = time(cnt:cntEnd);
                else
                    Talus_zsw = Tzsw(cnt:cntEnd+20);
                    Talus_dzsw = Tdzsw(cnt:cntEnd+20);
                    tsw = time(cnt:cntEnd+20);
                end
                if cntEnd - cnt < 60
                    cnt = cnt + 1;
                    continue;
                end
                
                
                tsw_norm = (tsw - tsw(1))/(tsw(end) - tsw(1));
                Talus_zsw_norm = interp1(tsw_norm,Talus_zsw,linspace(0,1,100));
                Talus_dzsw_norm = interp1(tsw_norm,Talus_dzsw,linspace(0,1,100));
                zsw_norm_array = [zsw_norm_array;
                                  Talus_zsw_norm];
                dzsw_norm_array = [dzsw_norm_array;
                                   Talus_dzsw_norm];
                cnt_norm_array = [cnt_norm_array;
                                  cnt];
                              
                zsw_time_array{end+1} = Talus_zsw;
                dzsw_time_array{end+1} = Talus_dzsw;
                time_array{end+1} = tsw - tsw(1);
                % due to the overlap of DSP, we need to go slightly back to
                % detect again for the other leg
                cnt = cnt + round(cntEnd/2);
                xst_static(cnt:cntEnd) = 0;
            end
            cnt = cnt + 1;
        end
        cnt = 1;
        while cnt < length(xst_static)
            if xsw_static(cnt) == 1     % swing
                cntEnd = cnt + find(xsw_static(cnt:end)==0,1,'first');
                if isempty(cntEnd)
                    cntEnd = length(xsw_static);
                    Talus_zsw = Tzst(cnt:cntEnd);
                    Talus_dzsw = Tdzst(cnt:cntEnd);
                    tsw = time(cnt:cntEnd);
                else
                    Talus_zsw = Tzst(cnt:cntEnd+20);
                    Talus_dzsw = Tdzst(cnt:cntEnd+20);
                    tsw = time(cnt:cntEnd+20);
                end
                if cntEnd - cnt < 60 
                    cnt = cnt + 1;
                    continue;
                end
                
                tsw_norm = (tsw - tsw(1))/(tsw(end) - tsw(1));
                Talus_zsw_norm = interp1(tsw_norm,Talus_zsw,linspace(0,1,100));
                Talus_dzsw_norm = interp1(tsw_norm,Talus_dzsw,linspace(0,1,100));
                zsw_norm_array = [zsw_norm_array;
                                  Talus_zsw_norm];
                dzsw_norm_array = [dzsw_norm_array;
                                   Talus_dzsw_norm];
                cnt_norm_array = [cnt_norm_array;
                                  cnt];
                              
                zsw_time_array{end+1} = Talus_zsw;
                dzsw_time_array{end+1} = Talus_dzsw;
                time_array{end+1} = tsw - tsw(1);
                % due to the overlap of DSP, we need to go slightly back to
                % detect again for the other leg
                cnt = cnt + round(cntEnd/2);
                xst_static(cnt:cntEnd) = 0;
            end
            cnt = cnt + 1;
        end
        
        [~,I] = sort(cnt_norm_array);
        zsw_norm_array = zsw_norm_array(I,:);
        zsw1 = [zsw1; zsw_norm_array(1,:)];
        zsw2 = [zsw2; zsw_norm_array(2,:)];
        zsw3 = [zsw3; zsw_norm_array(3,:)];
        zsw4 = [zsw4; zsw_norm_array(4,:)];
        
        dzsw_norm_array = dzsw_norm_array(I,:);
        dzsw1 = [dzsw1; dzsw_norm_array(1,:)];
        dzsw2 = [dzsw2; dzsw_norm_array(2,:)];
        dzsw3 = [dzsw3; dzsw_norm_array(3,:)];
        dzsw4 = [dzsw4; dzsw_norm_array(4,:)];
        
        zsw_time_array = zsw_time_array(I);
        zswt1{end+1} = zsw_time_array{1};
        zswt2{end+1} = zsw_time_array{2};
        zswt3{end+1} = zsw_time_array{3};
        zswt4{end+1} = zsw_time_array{4};
        dzsw_time_array = dzsw_time_array(I);
        dzswt1{end+1} = dzsw_time_array{1};
        dzswt2{end+1} = dzsw_time_array{2};
        dzswt3{end+1} = dzsw_time_array{3};
        dzswt4{end+1} = dzsw_time_array{4};
        time_array = time_array(I);
        time1{end+1} = time_array{1};
        time2{end+1} = time_array{2};
        time3{end+1} = time_array{3};
        time4{end+1} = time_array{4};
        
%         figure; hold on; grid on;
%         for iii = 1:size(zsw_norm_array,1)
%             plot(zsw_norm_array(iii,:))
%         end 
        
        
    end % trial
    Lstep_struct.(exp).steps = Lstep;
    Lstep_struct.(exp).steps_std = std(Lstep,0,1);
    Lstep_struct.(exp).steps_mean = mean(Lstep,1);
    Lstep_struct.(exp).stepsTimes = Tstep;
    Lstep_struct.(exp).stepsTimes_std = std(Tstep,0,1);
    Lstep_struct.(exp).stepsTimes_mean = mean(Tstep,1);
    
    zsw_struct.(exp).zsw{1} = zsw1;
    zsw_struct.(exp).zsw_std{1} = std(zsw1,0,1);
    zsw_struct.(exp).zsw_mean{1} = mean(zsw1,1);
    zsw_struct.(exp).zsw{2} = zsw2;
    zsw_struct.(exp).zsw_std{2} = std(zsw2,0,1);
    zsw_struct.(exp).zsw_mean{2} = mean(zsw2,1);
    zsw_struct.(exp).zsw{3} = zsw3;
    zsw_struct.(exp).zsw_std{3} = std(zsw3,0,1);
    zsw_struct.(exp).zsw_mean{3} = mean(zsw3,1);
    zsw_struct.(exp).zsw{4} = zsw4;
    zsw_struct.(exp).zsw_std{4} = std(zsw4,0,1);
    zsw_struct.(exp).zsw_mean{4} = mean(zsw4,1);
    
    zsw_struct.(exp).dzsw{1} = dzsw1;
    zsw_struct.(exp).dzsw_std{1} = std(dzsw1,0,1);
    zsw_struct.(exp).dzsw_mean{1} = mean(dzsw1,1);
    zsw_struct.(exp).dzsw{2} = dzsw2;
    zsw_struct.(exp).dzsw_std{2} = std(dzsw2,0,1);
    zsw_struct.(exp).dzsw_mean{2} = mean(dzsw2,1);
    zsw_struct.(exp).dzsw{3} = dzsw3;
    zsw_struct.(exp).dzsw_std{3} = std(dzsw3,0,1);
    zsw_struct.(exp).dzsw_mean{3} = mean(dzsw3,1);
    zsw_struct.(exp).dzsw{4} = dzsw4;
    zsw_struct.(exp).dzsw_std{4} = std(dzsw4,0,1);
    zsw_struct.(exp).dzsw_mean{4} = mean(dzsw4,1);
    
    zsw_struct.(exp).zsw_time{1} = zswt1;
    zsw_struct.(exp).dzsw_time{1} = dzswt1;
    zsw_struct.(exp).time{1} = time1;
    zsw_struct.(exp).zsw_time{2} = zswt2;
    zsw_struct.(exp).dzsw_time{2} = dzswt2;
    zsw_struct.(exp).time{2} = time2;
    zsw_struct.(exp).zsw_time{3} = zswt3;
    zsw_struct.(exp).dzsw_time{3} = dzswt3;
    zsw_struct.(exp).time{3} = time3;
    zsw_struct.(exp).zsw_time{4} = zswt4;
    zsw_struct.(exp).dzsw_time{4} = dzswt4;
    zsw_struct.(exp).time{4} = time4;
%     figure; hold on; grid on;
%     plot(xst)
%     plot(xsw)
%     plot(xst_static)
%     plot(xsw_static)
%     legend('xst','xsw','xst_{var}','xsw_{var}')
end

save('data/human/Lstep_struct.mat','Lstep_struct')

%% Plotting steptimes
phs = fieldnames(Lstep_struct);

figure
subplot(1,2,1); hold on; grid on;
xlocs = [1 2 3 4 5 2 3 4 5];
for ph = [1 2 3 4 5]
    plot(xlocs(ph)*ones(8,1),...
         Lstep_struct.(phs{ph}).stepsTimes(:,1),'ro')
    plot(xlocs(ph),mean(Lstep_struct.(phs{ph}).stepsTimes(:,1)),'bo')
end

subplot(1,2,2); hold on; grid on;
for ph = [1 6 7 8 9]
    plot(xlocs(ph)*ones(8,1),...
         Lstep_struct.(phs{ph}).stepsTimes(:,1),'ro')
    plot(xlocs(ph),mean(Lstep_struct.(phs{ph}).stepsTimes(:,1)),'bo')
end
linkaxes

%% Plotting stepsize
figure; 
%%%%%%%%%%%%%
%%% stepsizes 
subplot(2,3,1); hold on; grid on;
for i = 1:8
    plot(1:size(Lstep_struct.exp00.steps,2),...
         Lstep_struct.exp00.steps(i,:),'ro')
end
p = plot(1:size(Lstep_struct.exp00.steps,2),...
         Lstep_struct.exp00.steps_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
ylabel('stepsize [m]')
xticks([1 2 3 4])
xticklabels({'nom','ds','os','us'})
title('exp00')

subplot(2,3,2); hold on; grid on;
for i = 1:8
    plot(1:size(Lstep_struct.exp100.steps,2),...
         Lstep_struct.exp100.steps(i,:),'ro')
end
p = plot(1:size(Lstep_struct.exp100.steps,2),...
         Lstep_struct.exp100.steps_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
ylabel('stepsize [m]')
xticks([1 2 3 4])
xticklabels({'nom','ds','os','us'})
title('exp100')

subplot(2,3,3); hold on; grid on;
for i = 1:8
    plot(1:size(Lstep_struct.unexp100.steps,2),...
         Lstep_struct.unexp100.steps(i,:),'ro')
end
p = plot(1:size(Lstep_struct.unexp100.steps,2),...
         Lstep_struct.unexp100.steps_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
ylabel('stepsize [m]')
xticks([1 2 3 4])
xticklabels({'nom','ds','os','us'})
title('unexp100')

%%%%%%%%%%%%%
%%% steptimes
subplot(2,3,4); hold on; grid on;
for i = 1:8
    plot(1.5:size(Lstep_struct.exp00.stepsTimes,2)+0.5,...
         Lstep_struct.exp00.stepsTimes(i,:),'ro')
end
p = plot(1.5:size(Lstep_struct.exp00.stepsTimes,2)+0.5,...
         Lstep_struct.exp00.stepsTimes_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
xticks([1.5 2.5 3.5])
xticklabels({'ds','os','us'})
ylabel('steptime [s]')

subplot(2,3,5); hold on; grid on;
for i = 1:8
    plot(1.5:size(Lstep_struct.exp100.stepsTimes,2)+0.5,...
         Lstep_struct.exp100.stepsTimes(i,:),'ro')
end
p = plot(1.5:size(Lstep_struct.exp100.stepsTimes,2)+0.5,...
         Lstep_struct.exp100.stepsTimes_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
xticks([1.5 2.5 3.5])
xticklabels({'ds','os','us'})
ylabel('steptime [s]')

subplot(2,3,6); hold on; grid on;
for i = 1:8
    plot(1.5:size(Lstep_struct.unexp100.stepsTimes,2)+0.5,...
         Lstep_struct.unexp100.stepsTimes(i,:),'ro')
end
p = plot(1.5:size(Lstep_struct.unexp100.stepsTimes,2)+0.5,...
         Lstep_struct.unexp100.stepsTimes_mean,'bo');
p.MarkerSize = 10;
p.MarkerFaceColor = 'b';
xlim([0 5])
ylim([0 1])
xticks([1.5 2.5 3.5])
xticklabels({'ds','os','us'})
ylabel('steptime [s]')


%% Plotting zsw
figure
for i = 1:4
    subplot(3,4,i); hold on; grid on;
    for ii = 1:8
        plot(linspace(0,1,100),zsw_struct.exp00.zsw{i}(ii,:),'r')
    end
    plot(linspace(0,1,100),zsw_struct.exp00.zsw_mean{i},'b','LineWidth',3)
end
title('exp00')

for i = 1:4
    subplot(3,4,4+i); hold on; grid on;
    for ii = 1:8
        plot(linspace(0,1,100),zsw_struct.exp100.zsw{i}(ii,:),'r')
    end
    plot(linspace(0,1,100),zsw_struct.exp100.zsw_mean{i},'b','LineWidth',3)
end
title('exp100')

for i = 1:4
    subplot(3,4,4+4+i); hold on; grid on;
    for ii = 1:8
        plot(linspace(0,1,100),zsw_struct.unexp100.zsw{i}(ii,:),'r')
    end
    plot(linspace(0,1,100),zsw_struct.unexp100.zsw_mean{i},'b','LineWidth',3)
end
title('unexp100')
linkaxes


%% zsw
titles = {'Downstep','Overstep','Upstep','Nominal'};
figure
for i = 1:4
    subplot(1,4,i); hold on; grid on;
    plot(linspace(0,1,100),zsw_struct.exp00.zsw_mean{i},'LineWidth',2)
    plot(linspace(0,1,100),zsw_struct.exp100.zsw_mean{i},'LineWidth',2)
    plot(linspace(0,1,100),zsw_struct.unexp100.zsw_mean{i},'LineWidth',2)
    legend('exp00','exp100','unexp100')
    title(titles{i})
end
linkaxes

%% dzsw
titles = {'dDownstep','dOverstep','dUpstep','dNominal'};
figure
for i = 1:4
    subplot(1,4,i); hold on; grid on;
    plot(linspace(0,1,100),zsw_struct.exp00.dzsw_mean{i},'LineWidth',2)
    plot(linspace(0,1,100),zsw_struct.exp100.dzsw_mean{i},'LineWidth',2)
    plot(linspace(0,1,100),zsw_struct.unexp100.dzsw_mean{i},'LineWidth',2)
    legend('exp00','exp100','unexp100')
    title(titles{i})
end
linkaxes


%% Raw mean steptimes
figure
for i = 1:4
    subplot(3,4,i); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.exp00.time{i}{ii},zsw_struct.exp00.zsw_time{i}{ii});
        title(strcat('exp00 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('zsw [m]')
    end
    
    subplot(3,4,i+4); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.unexp100.time{i}{ii},zsw_struct.unexp100.zsw_time{i}{ii});
        title(strcat('unexp100 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('zsw [m]')
    end
    
    subplot(3,4,i+8); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.exp100.time{i}{ii},zsw_struct.exp100.zsw_time{i}{ii});
        title(strcat('exp100 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('zsw [m]')
    end
end
linkaxes
sgtitle('Raw zsw as a function of time')

figure
for i = 1:4
    subplot(3,4,i); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.exp00.time{i}{ii},zsw_struct.exp00.dzsw_time{i}{ii});
        title(strcat('exp00 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('dzsw [m]')
    end
    
    subplot(3,4,i+4); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.unexp100.time{i}{ii},zsw_struct.unexp100.dzsw_time{i}{ii});
        title(strcat('unexp100 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('dzsw [m]')
    end
    
    subplot(3,4,i+8); hold on; grid on;
    for ii = 1:8
        plot(zsw_struct.exp100.time{i}{ii},zsw_struct.exp100.dzsw_time{i}{ii});
        title(strcat('exp100 phase:',num2str(i)))
        xlabel('time [s]'); ylabel('dzsw [m]')
    end
end
linkaxes
sgtitle('Raw dzsw as a function of time')

%% Helper functions
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
    idx = find(contains(data.headersCoM,name));
end

function idx = getIdxDyn(data,name)
    idx = find(contains(data.exp00{1}.ID.headersDyn,name));
end