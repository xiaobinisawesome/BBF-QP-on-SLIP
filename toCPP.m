clear all; close all; clc;

load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierFInterpolationHuman.mat');
load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierZcomInterpolationCassie.mat');
load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/nominalBeziersCassie.mat');

%% Pre-print Fdes
% fid = fopen('/home/none/CassieHardware/src/cassie_controllers/include/cassie_controllers/gen/getBeziersFInterpolation.hpp','w');

% fprintf(fid,'#include <cassie_controllers/OUTPUT_HUMAN.hpp>\n');
% fprintf(fid,'#ifndef beziersF_HPP\n');
% fprintf(fid,'#define beziersF_HPP\n\n\n');
% fprintf(fid,'void OUTPUT_HUMAN::getBeziersFInterpolation(){\n');
% 
% %% Data-print
% % expected
% exps = {'exp','unexp'};
% phs  = {'phase1','phase2','phase3'};
% grfs = {'grf_SSP','grf_DSP_st','grf_DSP_sw'};
% 
% for i = 1:length(exps)
%     exp = exps{i};
%     for ii = 1:length(phs)
%         ph = phs{ii};
%         for iii = 1:length(grfs)
%             grf = grfs{iii};
%             % each bezier point
%             for iv = 1:length(bezierFInterpolation.(exp).(ph).(grf))
%                 vectorName = strcat(exp,ph,grf,num2str(iv));
%                 vectorValues = bezierFInterpolation.(exp).(ph).(grf){iv};
%                 fprintf(fid,strcat('Eigen::Vector2d\t',vectorName,'(2);\n'));
%                 fprintf(fid,strcat(vectorName,' << ',...
%                     num2str(vectorValues(1)),',',...
%                     num2str(vectorValues(2)),';\n'));
%                 % add to correct vector of Eigen::Vector2d's
%                 fprintf(fid,strcat('downstepBeziersF.',exp,'.',ph,'.',...
%                                    grf,'.push_back(',vectorName,');\n'));
%                 
%                 fprintf(fid,'\n\n');
%             end
%         end
%     end
% end
% 
% %% Post-print
% fprintf(fid,'};\n\n');
% fprintf(fid,'#endif');

%% Pre-print ZCOM
% fid = fopen('/home/none/CassieHardware/src/cassie_controllers/include/cassie_controllers/gen/getBeziersZInterpolation.hpp','w');
% 
% fprintf(fid,'#include <cassie_controllers/OUTPUT_HUMAN.hpp>\n');
% fprintf(fid,'#ifndef beziersZ_HPP\n');
% fprintf(fid,'#define beziersZ_HPP\n\n\n');
% fprintf(fid,'void OUTPUT_HUMAN::getBeziersZInterpolation(){\n');
% 
% % Data-print
% % expected
% exps = {'exp','unexp'};
% phs  = {'phase1','phase2','phase3'};
% grfs = {'zcom','dzcom'};
% 
% for i = 1:length(exps)
%     exp = exps{i};
%     for ii = 1:length(phs)
%         ph = phs{ii};
%         for iii = 1:length(grfs)
%             grf = grfs{iii};
%             % each bezier point
%             for iv = 1:length(bezierZcomInterpolation.(exp).(ph).(grf))
%                 vectorName = strcat(exp,ph,grf,num2str(iv));
%                 vectorValues = bezierZcomInterpolation.(exp).(ph).(grf){iv};
%                 fprintf(fid,strcat('Eigen::Vector2d\t',vectorName,'(2);\n'));
%                 fprintf(fid,strcat(vectorName,' << ',...
%                     num2str(vectorValues(1)),',',...
%                     num2str(vectorValues(2)),';\n'));
%                 % add to correct vector of Eigen::Vector2d's
%                 fprintf(fid,strcat('downstepBeziersZ.',exp,'.',ph,'.',...
%                                    grf,'.push_back(',vectorName,');\n'));
%                 
%                 fprintf(fid,'\n\n');
%             end
%         end
%     end
% end
% 
% % Post-print
% fprintf(fid,'};\n\n');
% fprintf(fid,'#endif');

%% Pre-print Nominal
fid = fopen('/home/none/CassieHardware/src/cassie_controllers/include/cassie_controllers/gen/getNominalBeziers.hpp','w');

fprintf(fid,'#include <cassie_controllers/OUTPUT_HUMAN.hpp>\n');
fprintf(fid,'#ifndef NOMINAL_BEZIERS_HPP\n');
fprintf(fid,'#define NOMINAL_BEZIERS_HPP\n\n\n');
fprintf(fid,'void OUTPUT_HUMAN::getNominalBeziers(){\n');

% Data-print
% expected
exps = {'exp','unexp'};
phs  = {'phase1','phase2','phase3'};
grfs = {'bv_zcom','bv_dzcom','bv_ddzcom',...
        'bv_grf_SSP','bv_grf_DSP_st','bv_grf_DSP_sw',...
        'bv_dgrf_SSP','bv_dgrf_DSP_st','bv_dgrf_DSP_sw'};

% each bezier point
for iv = 1:length(grfs)
    grf = grfs{iv};
    vectorName = strcat('nominalBeziers.',grf);
    vectorValues = nominalBeziers.(grf);
    
    fprintf(fid,strcat(vectorName,'.resize(',num2str(length(vectorValues)),');\n'));
    fprintf(fid,strcat(vectorName,' << ',num2str(vectorValues(1)),',\n'));
    for v = 2:length(vectorValues)-1
        fprintf(fid,strcat(num2str(vectorValues(v)),',\n'));
    end
    fprintf(fid,strcat(num2str(vectorValues(end)),';\n\n'));
end
grf = 'timeMax_SSP';
fprintf(fid,strcat('nominalBeziers.',grf,' = ',num2str(nominalBeziers.(grf)),';\n'));
grf = 'timeMax';
fprintf(fid,strcat('nominalBeziers.',grf,' = ',num2str(nominalBeziers.(grf)),';\n'));

% Post-print
fprintf(fid,'};\n\n');
fprintf(fid,'#endif');




%% Swingz behavior
% load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/swingZbehavior.mat');
% fid = fopen('/home/none/CassieHardware/src/cassie_controllers/include/cassie_controllers/gen/getSwingZbehavior.hpp','w');
% 
% fprintf(fid,'#include <cassie_controllers/OUTPUT_HUMAN.hpp>\n');
% fprintf(fid,'#ifndef getSwingZbehavior_HPP\n');
% fprintf(fid,'#define getSwingZbehavior_HPP\n\n\n');
% fprintf(fid,'void OUTPUT_HUMAN::getSwingZbehavior(){\n');
% 
% steps = {'NS','OS','US'};
% datas = {'t','z','dz'};
% for i = 1:length(steps)
%     step = steps{i};
%     for ii = 1:length(datas)
%         data = datas{ii};
%         realData = swingZbehavior.(step).(data);
%         arrayName = strcat('this->swingZbehavior.',step,'.',data);
%         fprintf(fid,strcat(arrayName,'.resize(',num2str(length(realData)),');\n'));
%         fprintf(fid,strcat(arrayName,'<<',num2str(realData(1)),',\n'));
%         for iii = 2:length(realData)-1
%             fprintf(fid,strcat(num2str(realData(iii)),',\n'));            
%         end
%         fprintf(fid,strcat(num2str(realData(end)),';\n\n'));   
%     end
% end
% 
% fprintf(fid,'};\n\n');
% fprintf(fid,'#endif');


