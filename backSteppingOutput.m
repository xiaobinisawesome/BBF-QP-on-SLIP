classdef backSteppingOutput
    %BACKSTEPPINGOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % logging
        timeNormFLog = []; %c
        timeNormZLog = []; %c
        
        isDownstep = false; %c
        expectedDownstep = false %c
        downstepStep = 0; %c
        knownDownstepHeight = 0; %c
        downstepHeightDetected = 0; %c
        
        FdesPre = [0; 0];
        timePre = 0.0;
        
        swingZbehavior %c
        swingZbehaviorOS %c
        swingZbehaviorUS %c
        
        NcontactLegs = 2; %c
        
        useIncreasingDeviation %c
        useHumanZ = true;
        useHumanF = true;
        stepsToTrueDesired %c
        stepCnt = 1; %c
        
        stepTime = 0.0; %c
        stepTimeVLO = 0.0; %c
        t_norm = 0.0;
        t_norm_VLO = 0.0;
        
        TD %c
        TS %c
        z0 %c
        
        mFrac %c
        zsw2f %c
        
        nominalBeziers %c
        downstepBeziersF %c
        downstepBeziersZ %c
        bezierStepTimes %c
        
        deltaNominal %c
        deltaDownstep %c
        
        polar = struct; % trash
    end
    
    methods
        function obj = backSteppingOutput(system)
            if isequal(system,'Human')
                tmpNBH = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/nominalBeziersHuman.mat');
                tmpDBHF = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierFInterpolationHuman.mat');
                tmpDBHZ = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierZcomInterpolationHuman.mat');
                tmpBST  = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierStepTimesHuman.mat');
                obj.nominalBeziers = tmpNBH.nominalBeziers;
                obj.downstepBeziersF  = tmpDBHF.bezierFInterpolation;
                obj.downstepBeziersZ  = tmpDBHZ.bezierZcomInterpolation;
                obj.bezierStepTimes   = tmpBST.bezierStepTimes;
                
                obj.mFrac = 1;                
                obj.deltaNominal = 0.8915 - 0.8788;
                obj.deltaDownstep = 0.8412 - 0.8298;
            else % 'cassie'
                tmpNBH = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/nominalBeziersCassie.mat');
                tmpDBHF = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierFInterpolationHuman.mat');
                tmpDBHZ = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierZcomInterpolationCassie.mat');
                tmpBST  = load('/home/none/repos/BBF-QP-on-SLIP/data/outputs/bezierStepTimesCassie.mat');
                obj.nominalBeziers = tmpNBH.nominalBeziers;
                obj.downstepBeziersF  = tmpDBHF.bezierFInterpolation;
                obj.downstepBeziersZ  = tmpDBHZ.bezierZcomInterpolation;
                obj.bezierStepTimes   = tmpBST.bezierStepTimes;

                obj.mFrac = 31 / (66.5138);
                obj.deltaNominal = 0.0; %0.1150 - (0.7266-0.7211);
                obj.deltaDownstep = 0.0; %0.0267 + (0.6936-0.6868);
            end
            SSPfrac = (obj.nominalBeziers.timeMax_SSP)/obj.nominalBeziers.timeMax;
%             obj.TS = SSPfrac*obj.nominalBeziers.timeMax;
%             obj.TD = (1-SSPfrac)*obj.nominalBeziers.timeMax;
            Ttotal = 0.4925;
            obj.TS = SSPfrac*Ttotal;
            obj.TD = (1-SSPfrac)*Ttotal;
            obj.z0 = mean(bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1));
            
            obj.useIncreasingDeviation = true;
            obj.stepsToTrueDesired = 5;
            
%             obj = obj.SwingFootZconstruct;
            obj.polar.x = 0.0;
            obj.polar.dx = 0.0;
        end
        
        function obj = update(obj,isDownstep,downstepStep,knownDownstepHeight,...
                                  downstepHeightDetected,NcontactLegs,stepCnt)
            obj.isDownstep = isDownstep;
            obj.downstepStep = downstepStep;
            obj.knownDownstepHeight = knownDownstepHeight;
            obj.downstepHeightDetected = downstepHeightDetected;
            obj.NcontactLegs = NcontactLegs;
            obj.stepCnt = stepCnt;
        end
        
        function obj = setTimeAndZsw(obj,stepTime,stepTimeVLO,zsw2f)
            obj.stepTime = stepTime;
            obj.stepTimeVLO = stepTimeVLO;
            obj.zsw2f = zsw2f;
        end
        
        function obj = setTD(obj,TD)
            obj.TD = TD;
        end
    end
        
    %% OUTPUT METHODS
    methods
        %%%%%%%%%%%%% F
        function [Fdes, dFdes] = getDesiredF(obj, t)
            if obj.isDownstep
                if obj.expectedDownstep
                    [Fdes, dFdes] = getDesiredFExpected(obj,t);
                else
                    [Fdes, dFdes] = getDesiredFUnexpected(obj,t);
%                     [Fdes, dFdes] = getDesiredFUnexpectedAsExpected(obj,t);
                end
            else
                [Fdes, dFdes] = getDesiredFNominal(obj,t);
            end
            % save Fdes for derivative for next iteration
            updatePre(obj,t);
        end
        function obj = updatePre(obj,t)
            if obj.NcontactLegs == 2
                obj.FdesPre = Fdes;
            else
                obj.FdesPre = [Fdes;0];
            end
            obj.timePre = t;
        end
        
        function [Fdes, dFdes] = getDesiredFNominal(obj,t)
            if obj.NcontactLegs == 1
                % SSP: Normalize time for bezier 
                tTmp = clamp(obj.stepTime,0.0,obj.nominalBeziers.timeMax_SSP);
                obj.t_norm = (tTmp - 0.0) / ...
                    (obj.nominalBeziers.timeMax_SSP - 0.0);
                obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                % eval beziers
                Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,obj.t_norm);
                dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,obj.t_norm);
            else
                % DSP: Normalize time for bezier, DSP from 0 to 1
                % 0.05 added to allow GRF to go below zero (by letting
                % the phase go slightly larger than 1)
                tTmp = clamp(obj.stepTime,0.0,obj.TD+obj.TS);
                obj.t_norm = (tTmp - obj.TS) / (obj.TD);
                obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                % eval beziers
                Fdes = zeros(2,1);
                dFdes = zeros(2,1);
                Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,obj.t_norm);
                Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,obj.t_norm);
                dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,obj.t_norm);
                dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,obj.t_norm);
            end
        end
        function [Fdes, dFdes] = getDesiredFExpected(obj,t)
            persistent passedDSP1 DSP1min DSP1max
            if isempty(passedDSP1)
                passedDSP1 = false;
                DSP1min = 0;
                DSP1max = 0;
            end
            persistent passedDSP2 DSP2min DSP2max totalDownstepTimeSSP
            if isempty(passedDSP2)
                passedDSP2 = false;
                DSP2min = 0;
                DSP2max = 0;
            end
            
            if obj.NcontactLegs == 1
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        [~,idx] = min(abs(obj.swingZbehavior.z - h));
                        nominalStepTime = obj.swingZbehavior.t(idx);
                        if isempty(totalDownstepTimeSSP)
                            totalDownstepTimeSSP = (nominalStepTime-obj.stepTime);
                        end
                        tTmp = clamp(obj.stepTimeVLO,0.0,totalDownstepTimeSSP);
                        obj.t_norm_VLO = (tTmp - 0.0) / (totalDownstepTimeSSP - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        tTmp = clamp(obj.stepTime,0.0,nominalStepTime);
                        obj.t_norm = (tTmp - 0.0) / (nominalStepTime - 0.0);

                        n = length(obj.downstepBeziersF.exp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase1.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > obj.TS
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        n = length(obj.downstepBeziersF.exp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > (obj.TS+obj.TD)
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/(obj.TS+obj.TD);
                        end
                        obj.t_norm_VLO = (obj.t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        n = length(obj.downstepBeziersF.exp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                end
                dFdes = (Fdes - obj.FdesPre(1))/(t - obj.timePre);
            else % DSP
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.knownDownstepHeight;
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        if ~passedDSP1 
                            % should be Bezier with $h$ 
                            DSP1min = obj.stepTime;
                            DSP1max = DSP1min + obj.TD;
                            passedDSP1 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP1min,DSP1max);
                        obj.t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        n = length(obj.downstepBeziersF.exp.phase1.grf_DSP_sw);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.exp.phase1.grf_DSP_sw{i},h);
                        end
                        n = length(obj.downstepBeziersF.exp.phase1.grf_DSP_st);
                        bv_grf_DSP_st = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.exp.phase1.grf_DSP_st{i},h);
                        end

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm,1)/(obj.TD);
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        if ~passedDSP2 
                            % should be Bezier with $h$ 
                            DSP2min = obj.stepTime;
                            DSP2max = DSP2min + obj.TD;
                            passedDSP2 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP2min,DSP2max);
                        obj.t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        n = length(obj.downstepBeziersF.exp.phase2.grf_DSP_sw);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_DSP_sw{i},h);
                        end
                        n = length(obj.downstepBeziersF.exp.phase1.grf_DSP_st);
                        bv_grf_DSP_st = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_DSP_st{i},h);
                        end

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm,1)/(obj.TD);
                    case 3
                        % THERE SHOULD NOT OCCUR A DSP PHASE 3
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        tTmp = clamp(obj.stepTime,0.0,obj.TS+obj.TD);
                        obj.t_norm = (tTmp - obj.TS) / (obj.TD);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,obj.t_norm);
                        dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,obj.t_norm);
                end
                dFdes = (Fdes - obj.FdesPre)./(t - obj.timePre);
            end
        end
        function [Fdes, dFdes] = getDesiredFUnexpected(obj,t)
            persistent passedDSP1 DSP1min DSP1max
            if isempty(passedDSP1)
                passedDSP1 = false;
                DSP1min = 0;
                DSP1max = 0;
            end
            persistent passedDSP2 DSP2min DSP2max
            if isempty(passedDSP2)
                passedDSP2 = false;
                DSP2min = 0;
                DSP2max = 0;
            end
            
            if obj.NcontactLegs == 1
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.zsw2f;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        tTmp = clamp(obj.stepTime,0.0,obj.TS);
                        obj.t_norm = (tTmp - 0.0) / (obj.TS - 0.0);
                        obj.t_norm_VLO = (obj.t_norm - 0.50)/(1.00 - 0.50);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_SSP{i},h);
                        end
%                         Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,1);
                        dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,obj.t_norm);
                        obj.downstepHeightDetected = obj.zsw2f;
                    case 2
                        phase = 'phase2';
                        h = obj.downstepHeightDetected;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.TS
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        n = length(obj.downstepBeziersF.unexp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm);
                        dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,obj.t_norm);
                    case 3
                        phase = 'phase3';
                        h = obj.downstepHeightDetected;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.TS + obj.TDun
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.t_norm_VLO = (obj.t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_SSP,obj.t_norm);
                end
                dFdes = (Fdes - obj.FdesPre(1))/(t - obj.timePre);
            else % DSP
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.downstepHeightDetected;
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        TDdownstep = obj.TD;
                        if ~passedDSP1 
                            DSP1min = obj.stepTime;
                            DSP1max = DSP1min + TDdownstep;
                            passedDSP1 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP1min,DSP1max);
                        obj.t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        n = length(obj.downstepBeziersF.unexp.phase1.grf_DSP_sw);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_DSP_sw{i},h);
                        end
                        n = length(obj.downstepBeziersF.unexp.phase1.grf_DSP_st);
                        bv_grf_DSP_st = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_DSP_st{i},h);
                        end

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm) - 20;
                        dFdes(1) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,obj.t_norm);
                        dFdes(2) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,obj.t_norm);
                    case 2
                        phase = 'phase2';
                        h = obj.downstepHeightDetected;
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        if ~passedDSP2 
                            % should be Bezier with $h$ 
                            DSP2min = obj.stepTime;
                            DSP2max = DSP2min + obj.TD;
                            passedDSP2 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP2min,DSP2max);
                        obj.t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        n = length(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw);
                        bv_grf_DSP_st = zeros(1,n);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_st{i},h);
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw{i},h);
                        end
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,obj.t_norm);
                        dFdes(2) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,obj.t_norm);
                end
                dFdes = (Fdes - obj.FdesPre)./(t - obj.timePre);
            end
        end
        function [Fdes, dFdes] = getDesiredFUnexpectedAsExpected(obj,t)
            persistent passedDSP1 DSP1min DSP1max
            if isempty(passedDSP1)
                passedDSP1 = false;
                DSP1min = 0;
                DSP1max = 0;
            end
            persistent passedDSP2 DSP2min DSP2max totalDownstepTimeSSP
            if isempty(passedDSP2)
                passedDSP2 = false;
                DSP2min = 0;
                DSP2max = 0;
            end
            
            if obj.NcontactLegs == 1
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        [~,idx] = min(abs(obj.swingZbehavior.z - h));
                        nominalStepTime = obj.swingZbehavior.t(idx);
                        if isempty(totalDownstepTimeSSP)
                            totalDownstepTimeSSP = (nominalStepTime-obj.TS/2);
                        end
                        tTmp = clamp(obj.stepTimeVLO,0.0,totalDownstepTimeSSP);
                        obj.t_norm_VLO = (tTmp - 0.0) / (totalDownstepTimeSSP - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        tTmp = clamp(obj.stepTime,0.0,nominalStepTime);
                        obj.t_norm = (tTmp - 0.0) / (nominalStepTime - 0.0);

                        n = length(obj.downstepBeziersF.unexp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.TS
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        n = length(obj.downstepBeziersF.unexp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.nominalBeziers.timeMax
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.t_norm_VLO = (obj.t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,obj.t_norm,1)/(obj.TD);
                end
            else % DSP
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.knownDownstepHeight;
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        if ~passedDSP1 
                            % should be Bezier with $h$ 
                            DSP1min = obj.stepTime;
                            DSP1max = DSP1min + obj.TD;
                            passedDSP1 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP1min,DSP1max);
                        obj.t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        n = length(obj.downstepBeziersF.unexp.phase1.grf_DSP_sw);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_DSP_sw{i},h);
                        end
                        n = length(obj.downstepBeziersF.unexp.phase1.grf_DSP_st);
                        bv_grf_DSP_st = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_DSP_st{i},h);
                        end

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm,1)/(obj.TD);
                        return;
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        if ~passedDSP2 
                            % should be Bezier with $h$ 
                            DSP2min = obj.stepTime;
                            DSP2max = DSP2min + obj.TD;
                            passedDSP2 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP2min,DSP2max);
                        obj.t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        n = length(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw{i},h);
                        end
                        n = length(obj.downstepBeziersF.unexp.phase1.grf_DSP_st);
                        bv_grf_DSP_st = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_st{i},h);
                        end

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,obj.t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,obj.t_norm,1)/(obj.TD);
                        return;
                    case 3
                        % THERE SHOULD NOT OCCUR A DSP PHASE 3
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        tTmp = clamp(obj.stepTime,0.0,obj.TS+obj.TD);
                        obj.t_norm = (tTmp - obj.TS) / (obj.TD);
                        obj.timeNormFLog = [obj.timeNormFLog; obj.t_norm];

                        % eval beziers
                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,obj.t_norm);
                        Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,obj.t_norm);
                        dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,obj.t_norm);
                        dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,obj.t_norm);
                        return;
                end
            end
        end
        
        
        %%%%%%%%%%%%% z
        function [zdes, dzdes, ddzdes] = getDesiredZ(obj, t)
            if obj.isDownstep
                % downstep getting
                if obj.expectedDownstep 
                    [zdes,dzdes,ddzdes] = getDesiredZExpected(obj,t);
                else
                    [zdes,dzdes,ddzdes] = getDesiredZUnexpected(obj,t);
%                     [zdes,dzdes,ddzdes] = getDesiredZUnexpectedAsExpected(obj,t);
                end
            else
                [zdes,dzdes,ddzdes] = getDesiredZNominal(obj,t);
            end
        end
        
        function [zdes, dzdes, ddzdes] = getDesiredZNominal(obj,t)            
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                if obj.stepTime > (obj.TD+obj.TS)
                    obj.t_norm = 1;
                else
                    obj.t_norm = obj.stepTime/(obj.TD+obj.TS);
                end
                obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm];

                if obj.stepTimeVLO > (obj.TD+obj.TS)
                    obj.t_norm_VLO = 1;
                else
                    obj.t_norm_VLO = obj.stepTimeVLO/(obj.TD+obj.TS);
                end
                
                zdesMean = mean(bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1));
                if obj.useIncreasingDeviation
                    % from constant height to complete human gait
                    factors = [0 0 0 linspace(0,1.0,obj.stepsToTrueDesired)];
                    % get the proper index of the factors, either within
                    % the range or at the end (actual desired)
                    index = length(factors);
                    if obj.stepCnt < length(factors)
                        index = obj.stepCnt;
                    end
                    zdes = factors(index)*(bezier2(obj.nominalBeziers.bv_zcom,obj.t_norm)-zdesMean) + zdesMean;
                    dzdes = factors(index)*bezier2(obj.nominalBeziers.bv_dzcom,obj.t_norm);
                    ddzdes = factors(index)*bezier2(obj.nominalBeziers.bv_ddzcom,obj.t_norm);
                else
                    factor = 0.5;
                    zdes = factor*(bezier2(obj.nominalBeziers.bv_zcom,obj.t_norm)-zdesMean) + zdesMean;
                    dzdes = factor*bezier2(obj.nominalBeziers.bv_dzcom,obj.t_norm);
                    ddzdes = 0.0;
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0.0;
            end
        end
        function [zdes, dzdes, ddzdes] = getDesiredZExpected(obj,t)
            persistent totalDownstepTime
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                %%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%
                %%% DOWNSTEPS
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.knownDownstepHeight;
                        [~,idx] = min(abs(obj.swingZbehavior.z - h));
                        nominalStepTime = obj.swingZbehavior.t(idx);
                        if isempty(totalDownstepTime)
                            totalDownstepTime = obj.TD + (nominalStepTime-obj.stepTime);
                        end

                        if obj.stepTimeVLO > totalDownstepTime
                            obj.t_norm_VLO = 1;
                        else 
                            obj.t_norm_VLO = obj.stepTimeVLO/totalDownstepTime;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase1.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase1.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.exp.phase1.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.exp.phase1.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal;
%                         dzdes = bezier2(bv_dzcom,obj.t_norm_VLO);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        return;
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;

                        if obj.stepTime > obj.nominalBeziers.timeMax
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.exp.phase2.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase2.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.exp.phase2.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.exp.phase2.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm) - obj.deltaNominal;
%                         dzdes = bezier2(obj.nominalBeziers.bv_dzcom,obj.t_norm);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,obj.t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm,1)/(obj.TS+obj.TD);
%                         ddzdes = bezier2(obj.nominalBeziers.bv_ddzcom,obj.t_norm);
                        return; 
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > (obj.TS/2)
                            obj.t_norm_VLO = 1;
                        else
                            obj.t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.exp.phase3.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase3.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.exp.phase3.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.exp.phase3.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal;
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        return;                            
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
        end
        function [zdes, dzdes, ddzdes] = getDesiredZUnexpected(obj,t)
            persistent passedSSP1 firstSSP1 timePassedSSP1 impactTime
            if isempty(passedSSP1)
                passedSSP1 = false;
                firstSSP1  = true;
                impactTime = 0;
            end
            
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                %%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%
                %%% DOWNSTEPS
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.zsw2f;

%                         load('data/outputs/bezierStepTimes.mat')
                        
                        if firstSSP1
                            timePassedSSP1 = obj.stepTimeVLO;
                            firstSSP1 = false;
                        end
                        if ~passedSSP1 && obj.NcontactLegs == 2 % first DSP pass
                            passedSSP1 = true;
                            impactTime = obj.stepTimeVLO;
                        end
                        TDdownstep = obj.TD;
                        if passedSSP1
                            timeMax = impactTime + TDdownstep;
                        else
                            timeMax = obj.stepTimeVLO + TDdownstep;
                        end

                        if obj.stepTimeVLO > timeMax
                            obj.t_norm_VLO = 1;
                        else 
                            obj.t_norm_VLO = obj.stepTimeVLO/timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase1.zcom);
                        bv_zcom = zeros(1,n); bv_zcom2 = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase1.zcom{i},h);
                            bv_zcom2(i) = polyval(obj.downstepBeziersZ.unexp.phase1.zcom{i},h-0.005);
                        end
                        n = length(obj.downstepBeziersZ.unexp.phase1.dzcom);
                        bv_dzcom = zeros(1,n); bv_dzcom2 = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase1.dzcom{i},h);
                            bv_dzcom2(i) = polyval(obj.downstepBeziersZ.unexp.phase1.dzcom{i},h-0.005);
                        end

                        % t_norm_Downstep from 0 to 1 over obj.TD duration
                        t_norm_Downstep = clamp(obj.stepTimeVLO,timePassedSSP1,timePassedSSP1+obj.TD);
                        t_norm_Downstep = (t_norm_Downstep - timePassedSSP1)/(obj.TD);
                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal - (1-t_norm_Downstep)*obj.deltaDownstep;
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(timeMax) + ...
                                (bezier2(bv_zcom,obj.t_norm_VLO)-bezier2(bv_zcom2,obj.t_norm_VLO))/0.005*obj.dzsw2f;
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(timeMax) + ...
                                 (bezier2(bv_dzcom,obj.t_norm_VLO)-bezier2(bv_dzcom2,obj.t_norm_VLO))/0.005*obj.dzsw2f;
                        % gradient velocity is the velocity that occurs due
                        % to traversing the surface
%                         gradientVelocity = (0.7981 - 0.8209)/(4.511 - 4.476);
%                         dzdes = gradientVelocity/2;
%                         dzdes = -0.1798;
%                         dzdes = bezier2(bv_dzcom,obj.t_norm_VLO);
%                         ddzdes = 0.0;
                        
                        if obj.stepTime > (obj.TD+obj.TS)
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/(obj.TD+obj.TS);
                        end
                        [zcomNominal,~,~] = obj.getDesiredZNominal(t);
                        obj.zcomNominalLog = [obj.zcomNominalLog;
                                              obj.xcom zcomNominal 0.0 obj.t_norm];
                        obj.zcomDownstepLog = [obj.zcomDownstepLog;
                                               obj.xcom zdes h obj.t_norm_VLO];
                        return;                            
                    case 2
                        phase = 'phase2';
                        h = obj.downstepHeightDetected;

                        if obj.stepTime > obj.nominalBeziers.timeMax
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.exp.phase2.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase2.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.exp.phase2.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase2.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm) - obj.deltaNominal;
%                             dzdes = bezier2(bv_dzcom,obj.t_norm);
                        dzdes = bezier2(bv_zcom,obj.t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm,1)/(obj.TS+obj.TD);
                        return;
                    case 3
                        phase = 'phase3';
                        h = obj.downstepHeightDetected;

                        if obj.stepTime > (obj.TS/2)
                            obj.t_norm_VLO = 1;
                        else
                            obj.t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase3.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase3.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.unexp.phase3.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase3.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal;
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(obj.TS/2);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(obj.TS/2);                           
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
        end
        function [zdes, dzdes, ddzdes] = getDesiredZUnexpectedAsExpected(obj,t)
            persistent totalDownstepTime firstSSP1 timePassedSSP1 
            if isempty(firstSSP1)
                firstSSP1 = true;
            end
            
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                %%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%
                %%% DOWNSTEPS
                switch obj.downstepStep
                    case 1
                        phase = 'phase1';
                        h = obj.zsw2f; %obj.knownDownstepHeight; % obj.zsw2f
                        if h == 0
                            h = obj.knownDownstepHeight;
                        end
                        
                        [~,idx] = min(abs(obj.swingZbehavior.z - h));
                        nominalStepTime = obj.swingZbehavior.t(idx);
                        if isempty(totalDownstepTime)
                            totalDownstepTime = (obj.TD) + (nominalStepTime-obj.TS/2);
                        end

                        if obj.stepTimeVLO > totalDownstepTime
                            obj.t_norm_VLO = 1;
                        else 
                            obj.t_norm_VLO = obj.stepTimeVLO/totalDownstepTime;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase1.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase1.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.unexp.phase1.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase1.dzcom{i},h);
                        end

                        if firstSSP1
                            timePassedSSP1 = obj.stepTimeVLO;
                            firstSSP1 = false;
                        end
                        % t_norm_Downstep from 0 to 1 over obj.TD duration
                        t_norm_Downstep = clamp(obj.stepTimeVLO,timePassedSSP1,timePassedSSP1+obj.TD);
                        t_norm_Downstep = (t_norm_Downstep - timePassedSSP1)/(obj.TD);
                        
                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal - (1-t_norm_Downstep)*obj.deltaDownstep;
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        return;
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;

                        if obj.stepTime > obj.TS+obj.TD
                            obj.t_norm = 1;
                        else
                            obj.t_norm = obj.stepTime/(obj.TS+obj.TD);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase2.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase2.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.unexp.phase2.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase2.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm) - obj.deltaNominal;
                        dzdes = bezier2(bv_zcom,obj.t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm,1)/(obj.TS+obj.TD);
                        return; 
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > (obj.TS/2)
                            obj.t_norm_VLO = 1;
                        else
                            obj.t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; obj.t_norm_VLO];

                        % get bezier polynomials from interpolation
                        n = length(obj.downstepBeziersZ.unexp.phase3.zcom);
                        bv_zcom = zeros(1,n);
                        for i = 1:n
                            bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase3.zcom{i},h);
                        end
                        n = length(obj.downstepBeziersZ.unexp.phase3.dzcom);
                        bv_dzcom = zeros(1,n);
                        for i = 1:n
                            bv_dzcom(i) = polyval(obj.downstepBeziersZ.unexp.phase3.dzcom{i},h);
                        end

                        % evaluate
                        zdes = bezier2(bv_zcom,obj.t_norm_VLO) - obj.deltaNominal + (obj.t_norm_VLO)*(0.7271-0.7338);
                        dzdes = bezier2(bv_zcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,obj.t_norm_VLO,1)/(totalDownstepTime);
                        return;                            
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
        end
       
        
        function obj = SwingFootZconstruct(obj)
            %% with bezier expected downstep
            Tf = obj.TS;
            xData = [0 0.2 0.4 0.5 0.6];
%             zData = [0 0.07 0 -0.3 -0.8];
            zData = [0 0.07 0 -0.1 -0.3];
            teval = linspace(0,0.6,100);
            
            zsw_max = 0.07;
            zsw_neg = -0.001;
            bv0 = [0 zsw_max zsw_neg -0.05 -0.3];
            fun = @(bv) rms(zData - bezier2(bv,xData));
            
            Aneq = [1 -1 0 0 0];
            bneq = [0];
            
            Aeq = [1 0 0 0 0];
            beq = [0];
            nlcon = @(bv) deal([max(bezier2(bv,teval)) - zsw_max],...
                                    [bezier2(bv,Tf) - zsw_neg;
                                     bezier2(bv,Tf/2) - zsw_max]);
            bv_zsw = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            figure; 
            subplot(1,3,1); hold on; grid on;
            plot(teval,bezier2(bv_zsw,teval))
            legend('original sinusoid','new bezier')
            
            swingZpos = bezier2(bv_zsw,teval);
            swingZvel = bezier2(bv_zsw,teval,1);
            obj.swingZbehavior = struct('t',teval,'z',swingZpos,'dz',swingZvel);
            
            %% Overstep
            xData = [0 0.2 0.4 0.5 0.6];
            zData = [0 0.07 0 -0.1 -0.3] - obj.knownDownstepHeight;
            teval = linspace(0,0.6,100);
            
            zsw_max = 0.07 - obj.knownDownstepHeight;
            zsw_neg = -0.001 - obj.knownDownstepHeight;
            bv0 = [0 zsw_max zsw_neg -0.05- obj.knownDownstepHeight -0.3- obj.knownDownstepHeight];
            fun = @(bv) rms(zData - bezier2(bv,xData));
            
            Aneq = [1 -1 0 0 0];
            bneq = [0];
            
            Aeq = [1 0 0 0 0];
            beq = [-obj.knownDownstepHeight];
            nlcon = @(bv) deal([max(bezier2(bv,teval)) - zsw_max],...
                                    [bezier2(bv,Tf) - zsw_neg;
                                     bezier2(bv,Tf/2) - zsw_max]);
            bv_zsw = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            subplot(1,3,2); hold on; grid on;
            plot(teval,bezier2(bv_zsw,teval))
            legend('new bezier')
            
            swingZpos = bezier2(bv_zsw,teval);
            swingZvel = bezier2(bv_zsw,teval,1);
            obj.swingZbehaviorOS = struct('t',teval,'z',swingZpos,'dz',swingZvel);
            
            %% Upstep
            xData = [0 0.2 0.4 0.5 0.6];
            zData = [obj.knownDownstepHeight 0.07 0 -0.1 -0.3] - obj.knownDownstepHeight;
            teval = linspace(0,0.6,100);
            
            zsw_max = 0.07;
            zsw_neg = -0.001;
            bv0 = [0 zsw_max zsw_neg -0.05- obj.knownDownstepHeight -0.3- obj.knownDownstepHeight];
            fun = @(bv) rms(zData - bezier2(bv,xData));
            
            Aneq = [1 -1 0 0 0];
            bneq = [0];
            
            Aeq = [1 0 0 0 0];
            beq = [obj.knownDownstepHeight];
            nlcon = @(bv) deal([max(bezier2(bv,teval)) - zsw_max],...
                                    [bezier2(bv,Tf) - zsw_neg;
                                     bezier2(bv,Tf/2) - zsw_max]);
            bv_zsw = fmincon(fun,bv0,Aneq,bneq,Aeq,beq,[],[],nlcon);
            
            subplot(1,3,3); hold on; grid on;
            plot(teval,bezier2(bv_zsw,teval))
            legend('new bezier')
            
            swingZpos = bezier2(bv_zsw,teval);
            swingZvel = bezier2(bv_zsw,teval,1);
            
            obj.swingZbehaviorUS = struct('t',teval,'z',swingZpos,'dz',swingZvel);
        end
    end
end

