classdef backSteppingWalking < handle

    properties
%         g = 9.81; 
%         m = 32; % Kg
%         Kparam = 8000; %% 8000 %%% in controller
%         Dparam = 100; %100; 
%         KparamSim = 8000; %%%% for simulation
%         DparamSim = 100; 
% 
%         K = @(L,Kparam) Kparam(1);
%         D = @(L,Dparam) Dparam(1);
        
        deltaNominal = 0.8915 - 0.8788;
        deltaDownstep = 0.8412 - 0.8298;
        
        % Length dependent stiffness and damping
        % now K is called as K(L,obj.Kparam) 
        g = 9.81;
        m = 66.5138;
        mFrac = 1;
        Kparam = [769.9856 911.3193 1.3764e4];
        Dparam = 100;
        KparamSim = [769.9856 911.3193 1.3764e4];
        DparamSim = 100;
        
        K = @(L,Kparam) polyval(Kparam,L);
        D = @(L,Dparam) polyval(Dparam,L);
        
        nominalBeziers;
        downstepBeziersF;
        downstepBeziersZ;
        bezierStepTimes;
        
        useIncreasingVelocity   = false;
        useIncreasingDeviation  = false;
        useDecreasingRelaxation = false;
        useNewNominalSpline     = false;
        
        useSwingFootDetection = false;
        xsw = 0.0;
        zsw = 0.0;
        xsw2f = 0.0;
        zsw2f = 0.0;
        dxsw2f = 0.0;
        dzsw2f = 0.0;
        xcom = 0.0;
        xcom2f = 0.0;              % xcom w.r.t. stance foot
        detectionHeight = -0.005;
        
        stepCnt = 1;
        stepTime = 0.0;
        stepTimeVLO = 0.0;
        passedVLO = false;
        stepsToTrueDesired = 8;
        useHumanZ = false;
        useHumanF = false;
        useTimeBased = true;
        exp = 'exp00';
        tLastVLO = 5.821;
        
        isDownstep = false;
        expectedDownstep = false;
        downstepStep = 0;
        downstepCompleted = false;
        downstepTime = 0.0;
        downstepDuration = 0.0;             % total time the procedure takes
        knownDownstepHeight = 0.0;          % actual downstep height for expected
        downstepHeightDetected = 0.0;       % detected downstep height after completing phase 1
        downstepHeight = 0.0;               % detected downstep height for unexpected
        knownDownstepStep = 12;
        
        VLOtime = 0.0;
        bv_SSP2 = struct();     % just for Fdes and dFdes of SSP2
        bv_SSP3 = struct();     % just for zcom and dzcom of SSP3
        zdesPrev = struct;
        FdesPrev = struct;
        FdesPre = [0; 0];
        dFdesPre = [0; 0];
        timePre = 0.0;
        
        %%% small D requires large leg length actuation; 
        %%% large D has large impact force, challenges the DSP
        odeopts = odeset('MaxStep', 1e-3, 'RelTol',1e-4, 'AbsTol',1e-4)
        X0;
        TS %% SSP duration
        TD %% DSP duration
        TF %%% simulation duration
        %%% log 
        Xsol =[];
        Tsol =[]
        usol =[]
        DSPt = [];      % time stamp for system in DSP
        GRFsol = [];
        GRFlr = []; 
        GRFsbound = [];     % stance
        GRFnsbound = [];    % non-stance
        GRFboundIdx = [];
        
        zdesBezier = [];      % desired zcom
        dzdesBezier = [];
        ddzdesBezier = [];
        zswBezier = [];
        
        isDownstepLog = [];
        downstepStepLog = [];
        NcontactLegsLog = [];
        stepCntLog = [];
        stepTimeLog = [];
        stepTimeVLOLog = [];
        totalStepTimeLog = [];
        timeNormFLog = [];
        timeNormZLog = [];
        zcomNominalLog = [];
        zcomDownstepLog = [];
        zswLog = [];
        zsw2fLog = [];
        xsw2fLog = [];
        qsLog = [];
        
        stepLdesOriginalLog = [];
        stepLdesTmpLog = [];
        
        
        FliftOff = 0;
        stepLengthSequence = 0;         
        TimeStamp = 0
        SLIPx = struct()

        %%% walking
        NcontactLegs %%% = 1  or 2
        stanceLegNum %
        stanceFootX = 0
        stanceFootZ = 0
        curstepLength = 0
        desiredbehavior
        polar = struct();% states in polar coord
        
        %%%%% behavior 
        swingHeight = 0.07; 
        impactVel = 0.1; 
        swingZbehavior = struct('z', [], 'dz', []);
        swingZbehaviorOS = struct('z', [], 'dz', []);
        swingZbehaviorUS = struct('z', [], 'dz', []);
        desiredGRF
        terrain
        z0
        
        terrainType %% Flat Slope Rough
   end
    
    properties  %% control
         %%% linear controller gain
        Kp = 10; %10 %%%% PD gains for leg length control
        Kd = 5; %5
        epsilon = 0.1; % 0.1;
        c_relax_DSP = 0.15; %0.3; %%% DSP, stance force relax coef
        c_relax_SSP = 0.15;
        c_relax_DSP_downstep = 0.15;
        c_relax_SSP_downstep = 0.15;
        deltaF = 50; % was 50
        Kbackstepping = 100; % was 100
        gamma = 10; % was 10 %%% clf : dV< -gamma*V; 
        beta = 500 % was 500 %%% CBF: dh > - beta*(h- Fmin) with h = Fs in SSP; %% larger 
        Fmin = 20;
        ddLmax = Inf; % was 500
        
        Fext = 0 %%% external push force
        LIP
        stepLdes = 0.0;
        dstepLdes = 0.0;
        maxStepsize = Inf;
        stepController = 'Deadbeat'; %'DeadbeatParallel'; %'Deadbeat'
        enablePushDisturbance = false;
        clfSol = struct('V', [], 'dV', [], 'delta', [], 'cbfH', [], 'exitflag', [])
        
        animData
        
        DiscreteX = struct('t', [], 'x', [], 'p', [], 'v', [], 'stepSize', []);
        newAvoid = 0; 
        dumpVec 
        smfVec 
        dsmfVec
    end 
    
    methods
        function obj = backSteppingWalking(system,exp)
            if isequal(system,'Human')
                tmpNBH = load('data/outputs/nominalBeziersHuman.mat');
                tmpDBHF = load('data/outputs/bezierFInterpolationHuman.mat');
                tmpDBHZ = load('data/outputs/bezierZcomInterpolationHuman.mat');
                tmpBST  = load('data/outputs/bezierStepTimesHuman.mat');
                obj.nominalBeziers = tmpNBH.nominalBeziers;
                obj.downstepBeziersF  = tmpDBHF.bezierFInterpolation;
                obj.downstepBeziersZ  = tmpDBHZ.bezierZcomInterpolation;
                obj.bezierStepTimes   = tmpBST.bezierStepTimes;
                
                obj.m = 66.5138;
                obj.mFrac = 1;
                obj.Kparam = [769.9856 911.3193 1.3764e4];
                obj.Dparam = 100;
                obj.KparamSim = obj.Kparam;
                obj.DparamSim = obj.Dparam;
                
                obj.deltaNominal = 0.0127;
                obj.deltaDownstep = 0.0162;
            else % 'cassie'
                tmpNBH = load('data/outputs/nominalBeziersCassie.mat');
                tmpDBHF = load('data/outputs/bezierFInterpolationHuman.mat');
                tmpDBHZ = load('data/outputs/bezierZcomInterpolationCassie.mat');
                tmpBST  = load('data/outputs/bezierStepTimesCassie.mat');
                obj.nominalBeziers = tmpNBH.nominalBeziers;
                obj.downstepBeziersF  = tmpDBHF.bezierFInterpolation;
                obj.downstepBeziersZ  = tmpDBHZ.bezierZcomInterpolation;
                obj.bezierStepTimes   = tmpBST.bezierStepTimes;
                
                obj.m = 31;
                obj.mFrac = obj.m / (66.5138);
                obj.Kparam = [23309 0 -55230 48657 -9451];
                obj.Dparam = [348 0 -824 726 -141];
                obj.KparamSim = obj.Kparam;
                obj.DparamSim = obj.Dparam;

                obj.deltaNominal = (0.9361-0.8189) + (0.7669-0.7564);
                obj.deltaDownstep = 0;
%                 obj.deltaDownstep = 0.8824 - 0.8557 - (0.8936 - 0.8824);
            end
            
            obj.exp = exp;
            
            SSPfrac = (obj.nominalBeziers.timeMax_SSP)/obj.nominalBeziers.timeMax;
            obj.TS = SSPfrac*obj.nominalBeziers.timeMax;
            obj.TD = (1-SSPfrac)*obj.nominalBeziers.timeMax;
            obj.z0 = mean(bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1));
            obj.TF = 10;     % total simulation time

            obj.terrain = terrainGen; 
            obj.LIP = LIPapprox(obj.TS, obj.TD, obj.z0);
            obj.dumpVec = 0:0.005:1;
            obj.smfVec = smf(obj.dumpVec, [obj.TS/3 obj.TS*2/3]);
            obj.dsmfVec = [0,diff(obj.smfVec)]/0.005;
        end
        
        function setVelocity(obj, vdes) 
            obj.LIP.setDesiredVelocity(vdes);
        end 
        
        function setVelocityFromBezier(obj)
            % we walk a bit slower in order to have xcom less affected by
            % the actuation on z
            dxcom = 0.9*mean(bezier2(obj.nominalBeziers.bv_dxcom,0:0.01:1)); 
            if obj.useIncreasingVelocity
                dxN = 0.5;      % starting velocity
                desiredVelocity = [dxN dxN linspace(dxN,dxcom,obj.stepsToTrueDesired)];
                % get the proper index of the factors, either within
                % the range or at the end (actual desired)
                index = length(desiredVelocity);
                    if obj.stepCnt < length(desiredVelocity)
                        index = obj.stepCnt;
                    end
                obj.LIP.setDesiredVelocity(desiredVelocity(index));
            else 
                obj.LIP.setDesiredVelocity(0.5);
            end
        end
        
        function setDuration(obj, TF)
            obj.TF = TF; 
        end
        
        function reset(obj)
            sD = obj.m*obj.g/obj.K(obj.z0,obj.Kparam);
            obj.X0 = [0         % x
                      0.0       % dx
                      obj.z0    % z
                      0         % dz
                      obj.z0+sD % L1
                      0.        % dL1
                      obj.z0    % L2
                      0         % dL2
                      sD        % sL1
                      0.0       % dsL1
                      0.0       % sL2
                      0.0];     % dsL2
            
            if obj.useHumanZ
                % If we use the human zcom, we reset the initial condition
                obj.X0(3) = bezier2(obj.nominalBeziers.bv_zcom,0.0);
            end
            
            
            obj.NcontactLegs = 1; 
            obj.stanceLegNum = -1; 
            obj.curstepLength = 0; 
            obj.Xsol = []; 
            obj.Tsol = []; 
            obj.usol = []; 
            obj.GRFsol = []; 
             obj.GRFlr = []; 
            obj.clfSol = struct('V', [], 'dV', [], 'delta', [], 'cbfH', [], 'exitflag', []);
            obj.stanceFootX = 0;  
            obj.stanceFootZ = 0; 
            obj.TimeStamp = 0; 
            obj.stepLengthSequence = 0; 
            
            %%% desired GRF in DSP
            t = 0:0.01:2*obj.TD;
            Fmax = 1:-2/(length(t)-1):-1;
            dFmax = -ones(size(t))/obj.TD;
            obj.desiredGRF = struct('t', t, 'Fmax', Fmax, 'dFmax', dFmax);
            obj.SwingFootZconstruct();
            obj.newAvoid = 0;
        end

        function obj = simBackStepping(obj)
            obj.reset();             
            X0 = obj.X0;
            obj.stateTransform(0, X0);
            stanceX = obj.polar.x - obj.polar.rs*sin(obj.polar.qs); 
            stanceZ = obj.polar.z - obj.polar.rs*cos(obj.polar.qs); 
            terrainZ = interp1(obj.terrain.x, obj.terrain.z, stanceX); 
            X0(3) = X0(3) - (stanceZ - terrainZ); 
            obj.stanceFootZ = terrainZ;  
         %   genDesiredbehaviorArbitrary(obj) ;
         
            Tf = obj.TF;
            period = 0.001;
            ts = 0.0;
            obj.stepTime = 0.0;
            for t = 0:period:Tf
                obj.stepTime = obj.stepTime + period;
                obj.stepTimeVLO = obj.stepTimeVLO + period;
                
                obj.stateTransform(t, X0); % update the polar states before calculate controls
                %u = obj.backStepping(t);
                [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = obj.QPbackSteppingCLFCBF(t);
                % log for control
                obj.clfSol.V = [obj.clfSol.V, V];
                obj.clfSol.dV = [obj.clfSol.dV, dV];
                obj.clfSol.delta = [obj.clfSol.delta, delta];
                obj.clfSol.cbfH = [obj.clfSol.cbfH, cbfH] ;
                obj.clfSol.exitflag = [obj.clfSol.exitflag, exitflag];
                options = odeset(obj.odeopts, 'Events', @(t,X) obj.guard(t, X));
                
                obj.addPushDisturbance(t); 
                [T, X] = ode45(@(t,X) obj.dynamics(t, X, u), [ts,t+period], X0, options);
                X0 = X(end,:)';   
                ts = T(end);    
                
                if obj.stanceLegNum == -1
                    obj.GRFlr = [obj.GRFlr, [obj.polar.Fs; obj.polar.Fns]];
                else
                    obj.GRFlr = [obj.GRFlr, [obj.polar.Fns; obj.polar.Fs]];
                end
                obj.getDSPtime(ts);
                
                % Test foot penetration and foot going through the 'floor'
                testFootPenetration = obj.ifSwingFootPenetrating(ts);     
                testFootNegative    = obj.ifSwingFootNegative(ts);

                if T(end) < t+period || testFootPenetration%%% event happened
                    X0 = obj.updateDomain(X0, ts);
                end
                
                if obj.polar.qs > 0 && obj.NcontactLegs == 1 && ~obj.passedVLO
                    obj.stepTimeVLO = 0.0;
                    obj.passedVLO = true;
                end
                
                if T(end) < t+period && obj.NcontactLegs == 1
                    obj.totalStepTimeLog = [obj.totalStepTimeLog obj.stepTime];
                    % reset time if from DSP to SSP
                    obj.stepTime = 0.0;
                    obj.passedVLO = false;
                    
                    % increase the step count
                    obj.stepCnt = obj.stepCnt + 1;
                    % reset the desired velocity based on tnow
                    if obj.useIncreasingVelocity
                        obj.setVelocityFromBezier();
                    end
                    % if we're at downstep, increase stepCnt for downstep
                    % as we first check to increase and then check if
                    % isDownstep, we only do this on the second iteration
                    if obj.isDownstep
                        if obj.downstepStep == 1
                            obj.downstepHeight = obj.knownDownstepHeight;
                        end
                        obj.downstepStep = obj.downstepStep + 1;
                    end
                end
                
                
%                 if ~obj.isDownstep && obj.stepCnt >= obj.knownDownstepStep && obj.passedVLO && ~obj.downstepCompleted
%                     obj.isDownstep = true;
%                     obj.downstepStep = 1;
%                     obj.downstepTime = ts;
%                 end
                if obj.expectedDownstep
                    if ~obj.isDownstep && obj.stepCnt >= obj.knownDownstepStep && obj.passedVLO && ~obj.downstepCompleted
                        obj.isDownstep = true;
                        obj.downstepStep = 1;
                        obj.downstepTime = ts;
                    end
                else
                    if ~obj.isDownstep && testFootNegative && obj.stepCnt >= obj.knownDownstepStep && obj.passedVLO && ~obj.downstepCompleted
                        % switch to true if this is the first time
                        if ~obj.isDownstep
                            obj.isDownstep = true;
                            obj.downstepStep = 1;
                            obj.downstepTime = ts;
                        end
                    end
                end
                
%                 if ~testFootNegative && obj.downstepStep == 3 && obj.NcontactLegs == 2
%                     % go back to normal if we've had three steps and the
%                     % swing foot is again positive
%                     obj.isDownstep = false;
%                     obj.downstepStep = 0;
%                 end
                
                Xaug = obj.stateAugmentation(ts, X0);
                
%                 if obj.downstepStep == 3 && obj.stepTime > 0.05 && obj.polar.qs > 0 && obj.NcontactLegs == 1
%                     % go back to normal if we've had three steps and the
%                     % swing foot is again positive
%                     obj.isDownstep = false;
%                     obj.downstepStep = 0;
%                     obj.downstepCompleted = true;
%                 end
%                 if obj.downstepStep == 3 && obj.stepTime > 0.2040 && obj.polar.qs > 0 && obj.NcontactLegs == 1
                if obj.downstepStep == 3 && obj.stepTime > 0.2040 && obj.NcontactLegs == 1
                    % go back to normal if we've had three steps and the
                    % swing foot is again positive
                    obj.isDownstep = false;
                    obj.downstepStep = 0;
                    obj.downstepCompleted = true;
                end

                obj.Xsol = [obj.Xsol; Xaug'];
                obj.Tsol = [obj.Tsol, T(end)];
                obj.usol = [obj.usol, u];
                obj.GRFsol = [obj.GRFsol, [obj.polar.Fs; obj.polar.Fns]];
                
                obj.zdesBezier   = [obj.zdesBezier; zdesStruct.zdes];
                obj.dzdesBezier  = [obj.dzdesBezier; zdesStruct.dzdes];
                obj.ddzdesBezier = [obj.ddzdesBezier; zdesStruct.ddzdes];
                obj.zswBezier    = [obj.zswBezier; obj.zsw];
                obj.isDownstepLog = [obj.isDownstepLog; obj.isDownstep];
                obj.downstepStepLog = [obj.downstepStepLog; obj.downstepStep];
                obj.NcontactLegsLog = [obj.NcontactLegsLog; obj.NcontactLegs];
                obj.stepTimeLog  = [obj.stepTimeLog; obj.stepTime];
                obj.stepCntLog   = [obj.stepCntLog; obj.stepCnt];
                obj.stepTimeVLOLog = [obj.stepTimeVLOLog; obj.stepTimeVLO];
                obj.zswLog = [obj.zswLog; obj.zsw];
                obj.zsw2fLog = [obj.zsw2fLog; obj.zsw2f];
                obj.xsw2fLog = [obj.xsw2fLog; obj.xsw2f];
                obj.qsLog = [obj.qsLog; obj.polar.qs];
                
                if obj.polar.x > obj.terrain.x(end) - 0.3
                    break
                end
            end
%             obj.plot;
            obj.plotControl;
        end
        
        function X0 = updateDomain(obj, X0, ts)
            obj.NcontactLegs = 3 - obj.NcontactLegs; %%% domain
            obj.TimeStamp = [obj.TimeStamp, ts];
            if obj.NcontactLegs == 1   %next domain is SSP
                obj.stanceLegNum = - obj.stanceLegNum;
                obj.LIP.stanceLegNum = obj.stanceLegNum;
                obj.stanceFootX = obj.stanceFootX + obj.curstepLength;
                obj.stanceFootZ = obj.polar.z - obj.polar.rns*cos(obj.polar.qns);
                obj.stepLengthSequence = [obj.stepLengthSequence, obj.curstepLength];
                obj.GRFboundIdx = [obj.GRFboundIdx, length(obj.GRFsbound)]; 
                
                obj.FdesPre = [0; 0];
                obj.timePre = 0.0;
            else %%% next domain is DSP
                %%%% preimpact event 
                %obj.LIP.MPC_controller('MPCtrackingVelocityP1', ts, [obj.polar.x2f;obj.polar.dx])%%% MPC on LIP 
                obj.LIP.updateLIPstates(ts); 
                obj.LIP.calculateNominalTrajPerStep(ts, obj.TS, obj.TD, obj.polar, obj.curstepLength) 
                obj.logDiscreteState(ts); 
                X0 = obj.virtualImpactEvent(X0, obj.curstepLength);
                obj.FliftOff = obj.polar.Fs; 
                
                obj.FdesPre = [obj.FdesPre(1); 0];
                obj.timePre = 0.0;
            end
        end
        
        function logDiscreteState(obj, ts) 
            obj.DiscreteX.t = [ obj.DiscreteX.t, ts]; 
            obj.DiscreteX.x = [obj.DiscreteX.x, obj.polar.x]; 
            obj.DiscreteX.p = [obj.DiscreteX.p, obj.polar.x2f]; 
            obj.DiscreteX.v = [obj.DiscreteX.v, obj.polar.dx]; 
        end 
        
        function getDSPtime(obj, ts)            
            if obj.NcontactLegs == 2
                obj.DSPt = [obj.DSPt, ts]; 
            end
        end 
        
        function addPushDisturbance(obj, t)
              if t>3 && t<3.3 && obj.enablePushDisturbance
                  deltaV = 0.5; 
                  acc = deltaV/0.3; 
                  obj.Fext = obj.m*acc; 
              else
                  obj.Fext = 0; 
              end 
        end
    end
    
    methods %%% dynamics
        function Xdot = dynamics(obj, t, X, u)
            %%% X = [r, L, s, dr, dL, ds]
            % obj.stateTransform(t, X); % update the polar states 

            x = X(1); xdot = X(2);
            z = X(3); zdot = X(4);
            L1 = X(5); dL1 = X(6); %original left leg
            L2 = X(7); dL2 = X(8);
            sL1 = X(9);  dsL1 = X(10);
            sL2 = X(11); dsL2 = X(12);
            
            % t = t + obj.t;
            xddot = (obj.polar.Fs*sin(obj.polar.qs) + obj.polar.Fns*sin(obj.polar.qns) + obj.Fext)/obj.m;
            zddot = (obj.polar.Fs*cos(obj.polar.qs) + obj.polar.Fns*cos(obj.polar.qns))/obj.m - obj.g;
            
            if obj.stanceLegNum == -1
                ddr1 = obj.polar.ddrs;
                ddr2 = obj.polar.ddrns;
            else
                ddr2 = obj.polar.ddrs;
                ddr1 = obj.polar.ddrns;
            end
            if obj.NcontactLegs == 2
                ddsL1 = u(1) - ddr1;
                ddsL2 = u(2) - ddr2;
            else  %%% swing spring to 0 
                coef = 100;
                if obj.stanceLegNum == -1
                    ddsL1 = u(1) - ddr1;
                    ddsL2 = -1/(obj.epsilon^2)*coef^2*sL2 - 1/obj.epsilon*coef*dsL2;
                else
                    ddsL2 = u(2) - ddr2;
                    ddsL1 = -1/(obj.epsilon^2)*coef^2*sL1 - 1/obj.epsilon*coef*dsL1;
                end
            end
            ddL1 = u(1);
            ddL2 = u(2);
            Xdot = [xdot; xddot; zdot; zddot; dL1; ddL1; dL2; ddL2; dsL1; ddsL1; dsL2; ddsL2];
            
            if z - obj.stanceFootZ < 0.5 % about to fall over.
                Xdot = zeros(size(Xdot));
            end
        end
        
        function [value, isterminal, direction] = guard(obj, t, X)
            %  t = obj.t + t;
            obj.stateTransform(t, X);
            if obj.NcontactLegs == 1
                % this is important to determine foot strike
                % can be time-based switching, phase based (x or angle)
                isterminal = 1;
                direction = -1;
                if t - obj.TimeStamp(end) > obj.TS/2
                    swingx = obj.polar.x + obj.polar.rns*(-sin(obj.polar.qns));
                    swingZ = obj.polar.z - obj.polar.rns*cos(obj.polar.qns);
                    if obj.stepCnt == obj.knownDownstepStep
                        terrainZ = obj.knownDownstepHeight;
                    else
                        terrainZ = 0.0;
                    end
%                     terrainZ = interp1(obj.terrain.x, obj.terrain.z, swingx);
                    value = swingZ - terrainZ;
                    if value < -0.002
                        disp('foot penetration')
                    end
                else
                    value = 1;
                end
            else
                % stance foot force goes to 0
                value = obj.polar.Fs;%- 1.5; % offset to avoid phase cross
                isterminal = 1;
                direction = -1;
            end
        end
        
        function X = virtualImpactEvent(obj, X, stepLength)
            % transition from SSP to DSP; the swing leg transits from free swinging to ankered
            % do this in the beginning of DSP
            %obj.stateTransform(t, X, stanceFootX, stepLength, varargin);
            %%%%% this transform should do the job, here we do it
            %%%%% explicitly
            x = X(1);    dx = X(2);  z = X(3);    dz = X(4);
            L1 = X(5);   dL1 = X(6); %original left leg
            L2 = X(7);   dL2 = X(8);
            sL1 = X(9);  dsL1 = X(10);
            sL2 = X(11); dsL2 = X(12);
            x2f = x - obj.stanceFootX;
            z2f = z - obj.stanceFootZ;
            r_nonstance = sqrt(z2f^2 + (stepLength - x2f)^2);
            dr_nonstance = (2*dx*(x2f - stepLength) + 2*dz*z2f)/2/sqrt(z2f^2 + (x2f - stepLength)^2);
            sL_nonstance = obj.polar.Lns - r_nonstance;
            dsL_nonstance = obj.polar.dLns - dr_nonstance;
            q_nonstance = asin((x2f - stepLength)/r_nonstance);
            dq_nonstance =  (dx - dr_nonstance*sin(q_nonstance) )/ ( r_nonstance*cos(q_nonstance));
            
            %%%% to do:update polar states
            obj.polar.x2f = x2f;
            obj.polar.dx = dx;
            % ds_ns dq_ns changes with impact
            if obj.stanceLegNum == 1
                X(10) = dsL_nonstance; % left leg is swing
            else
                X(12) = dsL_nonstance;
            end
        end
        
        function obj = stateTransform(obj, t, X)
            x = X(1);    dx = X(2);  z = X(3);    dz = X(4);
            L1 = X(5);   dL1 = X(6); %original left leg
            L2 = X(7);   dL2 = X(8);
            sL1 = X(9);  dsL1 = X(10);
            sL2 = X(11); dsL2 = X(12);
            x2f = x - obj.stanceFootX;
            z2f = z - obj.stanceFootZ;
            % stance leg length
            if obj.stanceLegNum == -1
                L_stance = L1;
                dL_stance = dL1;
                L_nonstance = L2;
                dL_nonstance = dL2;
                sL_stance = sL1;
                sL_nonstance = sL2;
                dsL_stance = dsL1;
                dsL_nonstance = dsL2;
            else
                L_stance = L2;
                dL_stance = dL2;
                L_nonstance = L1;
                dL_nonstance = dL1;
                sL_stance = sL2;
                sL_nonstance = sL1;
                dsL_stance = dsL2;
                dsL_nonstance = dsL1;
            end
            % stance leg states
            r_stance = sqrt(z2f^2 + x2f^2);
            dr_stance = (2*dx*x2f + 2*dz*z2f)/2/sqrt(z2f^2 + x2f^2);
            
            % stance leg angle
            q_stance = asin((x2f)/r_stance);
            dq_stance = (dx - dr_stance*sin(q_stance) )/ ( r_stance*cos(q_stance));
            
            if obj.NcontactLegs == 1
                [L, dL] = obj.StepController(t, x, x2f, z2f, dx);
                %  L = 0; dL = 0;
                obj.curstepLength = L;
                r_nonstance = L_nonstance - sL_nonstance;
                dr_nonstance = dL_nonstance - dsL_nonstance;
                sL_nonstance = 0;
                dsL_nonstance = 0;
                %%% hack again
                if ~isfield(obj.polar, 'x2f') %%%% if it is the first step
                    q_nonstance = 0; dq_nonstance = 0;
                else
                   [q_nonstance, dq_nonstance] = obj.stepLengthUpdateSwingLegAngle(L, dL, r_stance, q_stance, r_nonstance, ...
                     dr_stance, dq_stance, dr_nonstance  ); % update qns dqns
                  %  q_nonstance = 0; dq_nonstance = 0;
                end
              
            else %obj.NcontactLegs == 2 % stance leg in the previous SSP is the current stance leg in DSP
                stepLength = obj.curstepLength;
                r_nonstance = L_nonstance - sL_nonstance;
                dr_nonstance = dL_nonstance - dsL_nonstance;
                q_nonstance = asin((x2f - stepLength)/r_nonstance);
                dq_nonstance =  (dx - dr_nonstance*sin(q_nonstance) )/ ( r_nonstance*cos(q_nonstance));
            end
            
            Fstance = obj.springForceSim(L_stance,sL_stance,dsL_stance);
            if obj.NcontactLegs == 2
                Fnonstance = obj.springForceSim(L_nonstance,sL_nonstance,dsL_nonstance); % becomes 0 in SSP
                ddq_stance = 1/r_stance*(-dq_stance*dr_stance + obj.g*sin(q_stance)) ...
                    - Fnonstance/obj.m*sin(q_stance - q_nonstance);
                ddq_nonstance = 1/r_nonstance*(-dq_nonstance*dr_nonstance + obj.g*sin(q_nonstance)) + ...
                    Fstance/obj.m*sin(q_stance - q_nonstance);
            else
                Fnonstance = 0;
                ddq_stance = 1/r_stance*(-dq_stance*dr_stance + obj.g*sin(q_stance));
                ddq_nonstance = 0; % this won't be used anyway, and has not much sense due to the assumption that swing leg angle has no/trivial dynamics
            end
            
            ddr_stance = (Fstance + Fnonstance*cos(q_stance - q_nonstance))/obj.m - obj.g*cos(q_stance) + obj.Fext*sin(q_stance)/obj.m + r_stance*dq_stance^2;
            ddr_nonstance = (Fnonstance + Fstance*cos(q_stance - q_nonstance))/obj.m - obj.g*cos(q_nonstance) + obj.Fext*sin(q_nonstance)/obj.m + r_nonstance*dq_nonstance^2;
            
            obj.polar = struct('z', z, 'dz', dz, 'z2f', z2f, 'x', x, 'x2f', x2f, 'dx', dx, 'rs', r_stance,'drs', dr_stance,'ddrs',ddr_stance, 'qs', q_stance,'dqs', dq_stance, 'ddqs', ddq_stance, ...
                'Ls', L_stance,'sLs', sL_stance,'dLs', dL_stance,'dsLs', dsL_stance,'Fs',Fstance,...
                'rns', r_nonstance,'drns', dr_nonstance,'ddrns',ddr_nonstance, 'qns', q_nonstance,'dqns', dq_nonstance,'ddqns', ddq_nonstance, ...
                'Lns', L_nonstance,'sLns', sL_nonstance,'dLns', dL_nonstance,'dsLns', dsL_nonstance,'Fns',Fnonstance );
            obj.xsw = obj.polar.x - r_nonstance*sin(obj.polar.qns); 
            obj.zsw = obj.polar.z - r_nonstance*cos(obj.polar.qns);
            obj.xsw2f = r_stance*sin(obj.polar.qs) - r_nonstance*sin(obj.polar.qns);
            obj.zsw2f = r_stance*cos(obj.polar.qs) - r_nonstance*cos(obj.polar.qns);
            obj.dxsw2f = r_stance*cos(obj.polar.qs)*obj.polar.dqs + dr_stance*sin(obj.polar.qs) - ...
                         r_nonstance*cos(obj.polar.qns)*obj.polar.dqns - dr_nonstance*sin(obj.polar.qns);
            obj.dzsw2f = -r_stance*sin(obj.polar.qs)*obj.polar.dqs + dr_stance*sin(obj.polar.qs) + ...
                         r_nonstance*sin(obj.polar.qns)*obj.polar.dqns - dr_nonstance*cos(obj.polar.qns);
            obj.xcom = obj.polar.x;
            obj.xcom2f = x2f;
        end
        
        function Force = springForce(obj, L, s, ds)
            Force = obj.K(L,obj.Kparam)*s + obj.D(L,obj.Dparam)*ds;
        end
        
        function Force = springForceSim(obj, L, s, ds)
            Force = obj.K(L,obj.KparamSim)*s + obj.D(L,obj.DparamSim)*ds;
        end
        
        function Xsol = stateAugmentation(obj, Tsol, X)
            % augment the states after simulation in each domain
           % X = X';
            x = X(1,:);    dx = X(2,:);  z = X(3,:);    dz = X(4,:);
            L1 = X(5,:);   dL1 = X(6,:); %original left leg
            L2 = X(7,:);   dL2 = X(8,:);
            sL1 = X(9,:);  dsL1 = X(10,:);
            sL2 = X(11,:); dsL2 = X(12,:);
            
            Xsol = zeros(20, size(X,2));
            Xsol(1:12, :) = X;
            
            x2f = x - obj.stanceFootX;
            z2f = z - obj.stanceFootZ;
            % stance leg length
            if obj.stanceLegNum == -1
                L_stance = L1;
                dL_stance = dL1;
                L_nonstance = L2;
                dL_nonstance = dL2;
                sL_stance = sL1;         
                dsL_stance = dsL1;

                sL_nonstance = sL2;
                dsL_nonstance = dsL2;
            else
                L_stance = L2;
                dL_stance = dL2;
                L_nonstance = L1;
                dL_nonstance = dL1;
                sL_stance = sL2;     
                dsL_stance = dsL2;

                sL_nonstance = sL1;
                dsL_nonstance = dsL1;
            end
            % stance leg states
            r_stance = sqrt(z2f.^2 + x2f.^2);
            dr_stance = (2.*dx.*x2f + 2*dz.*z2f)./2./sqrt(z2f.^2 + x2f.^2);
            
            % stance leg angle
            q_stance = asin(x2f./r_stance);
            dq_stance = (dx - dr_stance.*sin(q_stance) )./ ( r_stance.*cos(q_stance));
            
            r_nonstance = L_nonstance - sL_nonstance;
            dr_nonstance = dL_nonstance - dsL_nonstance;
            % calcualte the q nonstance
            if obj.NcontactLegs == 1 % ~strcmp(obj.domain, 'DSP ') % ssp
                [L, dL] = obj.StepController(Tsol, x, x2f, z2f, dx);
                [q_nonstance, dq_nonstance] = obj.stepLengthUpdateSwingLegAngle(L, dL, ...
                    r_stance, q_stance, r_nonstance, dr_stance, dq_stance, dr_nonstance);
            else % if obj.NcontactLegs == 2 % stance leg in the previous SSP is the current stance leg in DSP
                q_nonstance = asin((x2f - obj.curstepLength)./r_nonstance);
                dq_nonstance =  (dx - dr_nonstance.*sin(q_nonstance) )./ ( r_nonstance.*cos(q_nonstance));
            end
            if obj.stanceLegNum == -1
                r1 = r_stance;      dr1 = dr_stance;
                r2 = r_nonstance;   dr2 = dr_nonstance;
                q1 = q_stance;      dq1 = dq_stance;
                q2 = q_nonstance;   dq2 = dq_nonstance;
            else
                r2 = r_stance;      dr2 = dr_stance;
                r1 = r_nonstance;   dr1 = dr_nonstance;
                q2 = q_stance;      dq2 = dq_stance;
                q1 = q_nonstance;   dq1 = dq_nonstance;
            end
            Xsol(13,:) = r1 ;
            Xsol(14,:) = dr1;
            Xsol(15,:) = r2;
            Xsol(16,:) = dr2;
            Xsol(17,:) = q1;
            Xsol(18,:) = dq1;
            Xsol(19,:) = q2;
            Xsol(20,:) = dq2;
            
            obj.zsw2f = r_stance*cos(q_stance) - r_nonstance*cos(q_nonstance);
            obj.xsw2f = r_stance*sin(q_stance) - r_nonstance*sin(q_nonstance);
        end
        
        function ifpen = ifSwingFootPenetrating(obj, t)
            ifpen = 0; 
            if obj.NcontactLegs== 1
                if t - obj.TimeStamp(end) > obj.TS/2
                    swingx = obj.polar.x + obj.polar.rns*(-sin(obj.polar.qns));
                    swingZ = obj.polar.z - obj.polar.rns*cos(obj.polar.qns);
                    if obj.stepCnt == obj.knownDownstepStep
                        terrainZ = obj.knownDownstepHeight;
                    else
                        terrainZ = 0.0;
                    end
%                     terrainZ = interp1(obj.terrain.x, obj.terrain.z, swingx);
                    value = swingZ - terrainZ;
                    if value < -1e-4 
                        ifpen = 1; 
                    end 
                end 
            end
        end
        
        function ifneg = ifSwingFootNegative(obj,t)
            ifneg = 0;
            if obj.NcontactLegs == 1
%                 if t - obj.TimeStamp(end) > obj.TS/2
                    if obj.zsw2f < obj.detectionHeight
                        ifneg = 1;
                    end
%                 end
            end
        end
    end
    
    methods %%% analysis
        function plot(obj)
            X = obj.Xsol;
            t = obj.Tsol;
            
            x = X(:,1);     dx = X(:,2);
            z = X(:,3);     dz = X(:,4);
            L1 = X(:,5);    dL1 = X(:,6); %original left leg
            L2 = X(:,7);    dL2 = X(:,8);
            sL1 = X(:,9);   dsL1 = X(:,10);
            sL2 = X(:,11);  dsL2 = X(:,12);
            r1 = X(:, 13);  dr1 = X(:, 14);
            r2 = X(:, 15);  dr2 = X(:, 16);
            q1 = X(:, 17);  dq1 = X(:, 18);
            q2 = X(:, 19);  dq2 = X(:, 20);
            
            %%%%%%%%%%% leg angles and zsw %%%%%%%%%%
            figure; 
            subplot(2,1,1); hold on; grid on;
            plot(t,obj.qsLog)
            title('stance leg angle')
            xlabel('time'); ylabel('qs')
            
            subplot(2,1,2); hold on; grid on;
            plot(t,obj.zswLog)
            plot(t,obj.zsw2fLog)
            title('zsw')
            xlabel('time'); ylabel('zsw')
            legend('zsw','zsw2f')
            
            %%%%%%%%%%% downstep phase %%%%%%%%%%%%
            figure; 
            subplot(2,1,1); hold on; grid on;
            plot(t,obj.isDownstepLog,'b')
            plot(t,obj.downstepStepLog,'r')
            plot(t,obj.NcontactLegsLog,'g')
            plot(t,obj.stepCntLog,'c')
            legend('isDownstep','downsepStep','NcontactLegs')

%             subplot(2,1,2); hold on; grid on;
%             plot(t,obj.timeNormZLog,'b')
%             plot(t,obj.timeNormFLog,'r')
%             legend('timeNormZ','timeNormF')
            
            %%%%%%%%%%% task %%%%%%%%%%%
      
            figure; 
            subplot(4,1,1); hold on; grid on;
            plot(t,obj.stepTimeLog,'r')
            xlabel('t'); ylabel('stepTime')
            title('stepTime during simulation')
            
            subplot(4,1,2); hold on; grid on;
            plot(t,obj.stepTimeVLOLog,'r')
            plot(t,q1)
            plot(t,q2)
            xlabel('t'); ylabel('stepTimeVLO')
            title('stepTimeVLO during simulation')
            
            subplot(4,1,3); hold on; grid on;
            plot(t,obj.timeNormFLog)
            xlabel('t'); ylabel('timeNormFLog')
            title('timeNormFLog')
            
            subplot(4,1,4); hold on; grid on;
            plot(t,obj.timeNormZLog)
            xlabel('t'); ylabel('timeNormZLog')
            title('timeNormZLog')
            
            
            figure,
            subplot(3,2,1); hold on; grid on;
            plot(t, x, 'r');
            title('x');
            xlim([0, t(end)])
            
            subplot(3,2,2); hold on; grid on;
            plot(t, dx, 'r');
            title('dx');
            xlim([0, t(end)])
            
            subplot(3,2,3); hold on; grid on;
            plot(t, z, 'r')
            title('z');
            xlim([0, t(end)])
            
            subplot(3,2,4); hold on; grid on;
            plot(t, dz, 'r')
            title('dz');
            xlim([0, t(end)])
            
            subplot(3,2,5); hold on; grid on;
            plot(x,z,'r')
            plot(obj.desiredbehavior.x, obj.desiredbehavior.zdes, 'b')
            plot(x,obj.zdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(x), max(x)])
            title('x-z');
            
            subplot(3,2,6); hold on; grid on;
            plot(x,dz,'r')
            plot(x,dx.*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x), 'b')
            plot(x,obj.dzdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(x), max(x)])
            title('x-dz')
            
            %%%%%%%%%%%% clear z, dz, and GRF tracking %%%%%%%%%%%%%%%%
            figure
            subplot(4,1,1); hold on; grid on;
            plot(t,dx,'r')
            title('t-dx');
            
            subplot(4,1,2); hold on; grid on;
            plot(t,z,'r')
            plot(obj.desiredbehavior.x, obj.desiredbehavior.zdes, 'b')
            plot(t,obj.zdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(t), max(t)])
            title('t-z');
            
            subplot(4,1,3); hold on; grid on;
            plot(t,dz,'r')
            plot(x,dx.*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x), 'b')
            plot(t,obj.dzdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(t), max(t)])
            title('t-dz')
            
            subplot(4,1,4); hold on; grid on;
            plot(obj.GRFsbound(:,1),obj.GRFsbound(:,2),'g')
            plot(obj.GRFsbound(:,1),obj.GRFsbound(:,3),'c')
            plot(obj.GRFsbound(:,1),obj.GRFsbound(:,4),'g')
            plot(obj.GRFnsbound(:,1),obj.GRFnsbound(:,2),'g')
            plot(obj.GRFnsbound(:,1),obj.GRFnsbound(:,3),'c')
            plot(obj.GRFnsbound(:,1),obj.GRFnsbound(:,4),'g')
            plot(t, obj.GRFsol(1,:), 'r')
            plot(t, obj.GRFsol(2,:), 'b') 
            xlim([min(t), max(t)])
            title('F (N)')
            legend('Fs', 'Fns', 'bound');

            %%%%%%%%%%%% leg behavior %%%%%%%%%%%%%
%             figure
%             subplot(3,2,1); hold on; grid on;
%             plot(t, sL1, 'r')
%             plot(t, sL2, 'b')
%             legend('sL1', 'sL2')
%             title('s');
%              
%             subplot(3,2,2); hold on; grid on;
%             plot(t, L1, 'r')
%             plot(t, L2, 'b')
%             title('L')
%             legend('L1', 'L2')
% 
%             subplot(3,2,3); hold on; grid on;
%             plot(t, dL1, 'r')
%             plot(t, dL2, 'b')
%             title('dL')
%             legend('dL1', 'dL2')
%             
%             subplot(3,2,4); hold on; grid on;
%             plot(t, dsL1, 'r')
%             plot(t, dsL2, 'b')
%             title('ds');
%             legend('dsL1', 'dsL2');
% 
%             subplot(3,2,5); hold on; grid on;
%             plot(t, obj.usol(1,:), 'r')
%             plot(t, obj.usol(2,:), 'b')
%             title('ddL')
%             legend('ddL1', 'ddL2');
% 
%             subplot(3,2,6); hold on; grid on;
%             plot(t, obj.GRFsol(1,:), 'r')
%             plot(t, obj.GRFsol(2,:), 'b') 
%             title('F (N)')
%             legend('Fs', 'Fns');
            unDeformedFootPosZ1 = z - L1.*cos(q1);
            unDeformedFootVelZ1 = dz - dL1.*cos(q1) + L1.*sin(q1).*dq1;
            
            unDeformedFootPosZ2 = z - L2.*cos(q2);
            unDeformedFootVelZ2 = dz - dL2.*cos(q2) + L2.*sin(q2).*dq2;
            
            %%%%%%%%%% feet positions %%%%%%%%%%%
            figure; 
            subplot(2,2,1); hold on; grid on;
            plot(t, unDeformedFootPosZ1, 'r')
            plot(t, unDeformedFootPosZ2, 'b')
            plot(obj.swingZbehavior.t, obj.swingZbehavior.z, 'k');
            title('undeformed swing pos')
            subplot(2,2,2); hold on; grid on;
            plot(t, unDeformedFootVelZ1, 'r')
            plot(t, unDeformedFootVelZ2, 'b')
            plot(obj.swingZbehavior.t, obj.swingZbehavior.dz, 'k');
            title('undeformed swing vel')
            
            FootPosZ1 = z - r1.*cos(q1);
            FootPosX1 = x - r1.*sin(q1);
            FootPosZ2 = z - r2.*cos(q2);
            FootPosX2 = x - r2.*sin(q2);

            h = subplot(2,2,3); hold(h, 'on');
            plot(obj.terrain.x, obj.terrain.z, 'k')
            plot(FootPosX1, FootPosZ1, 'r')
            plot(FootPosX2, FootPosZ2, 'b')
            title('FootPos XZ')
            h = subplot(2,2,4); hold(h, 'on');
            plot(t, FootPosZ1, 'r', t, FootPosZ2, 'b') 
            title('Foot PosZ vs t')
        end
        
        function plotLIP(obj)
            %             figure,
            %             N = length(obj.LIP.HLIP.t);
            %             subplot(3,1,1)
            %             plot(obj.LIP.HLIP.t, obj.LIP.HLIP.x2f) ;
            %             subplot(3,1,2)
            %             plot(obj.LIP.HLIP.t, obj.LIP.HLIP.dx);
            %             subplot(3,1,3)
            %             plot(obj.LIP.HLIP.x2f, obj.LIP.HLIP.dx) ;
            %
%             try
%                 figure,
%                 N = length(obj.LIP.HLIP.t);
%                 subplot(2,2,1)
%                 plot(obj.LIP.HLIP.t, obj.LIP.HLIP.x2f, 'r', obj.LIP.HLIP.t, obj.LIP.HLIP.pred_p, 'b'); 
%                 title('position')
%                 subplot(2,2,2)
%                 plot(obj.LIP.HLIP.t, obj.LIP.HLIP.dx, 'r', ...
%                     obj.LIP.HLIP.t, obj.LIP.HLIP.pred_v, 'b');            
%                 title('velocity')
%                 subplot(2,2,3)
%                 plot(obj.LIP.HLIP.x2f, obj.LIP.HLIP.dx) ; title('phase')
%                             subplot(2,2,4)
%                 plot(obj.LIP.HLIP.t, obj.LIP.HLIP.u, 'r'); title('step size') 
% 
%                 %%% discrete
%                 figure,
%                 subplot(3,1,1),
%                 title('p')
%                 plot(obj.LIP.robotXF.t, obj.LIP.robotXF.p, 'ro-', obj.LIP.robotXF.t, obj.LIP.XLIPlist(1,:), 'bo-');
%                 subplot(3,1,2),
%                 title('v')
%                 plot(obj.LIP.robotXF.t, obj.LIP.robotXF.v, 'ro-', obj.LIP.robotXF.t, obj.LIP.XLIPlist(2,:), 'bo-');
%                 subplot(3,1,3),           
%                 title('u')
%                 plot(obj.LIP.robotXF.t, obj.LIP.robotXF.u, 'ro-', obj.LIP.robotXF.t, obj.LIP.targetStepLengthVec, 'bo-');
%                 figure,        
%                 plot(obj.LIP.HLIP.t, obj.LIP.HLIP.u, 'r', obj.LIP.HLIP.t, obj.LIP.SLS_ulog, 'b'); title('step size') 
%             catch
%                 disp("not plotting something")
%             end
        end
        
        function plotleg(obj) 
                 X = obj.Xsol;
            t = obj.Tsol;
            usol = obj.usol;
            
            x = X(:,1);    dx = X(:,2);
            z = X(:,3);    dz = X(:,4);
            L1 = X(:,5);   dL1 = X(:,6); %original left leg
            L2 = X(:,7);   dL2 = X(:,8);
            sL1 = X(:,9);  dsL1 = X(:,10);
            sL2 = X(:,11); dsL2 = X(:,12);
            r1 =  X(:, 13); dr1 = X(:, 14);
            r2 = X(:, 15);  dr2 = X(:, 16);
            q1 = X(:, 17);  dq1 = X(:, 18);
            q2 = X(:, 19);  dq2 = X(:, 20);
            
            figure, 
            subplot(2,1,1); 
            plot(t, q1, 'r', t, q2,'b'); title('legs');
            subplot(2,1,2); 
            plot(t, dq1, 'r', t, dq2,'b'); title('dq legs')
        end 
        
        function plotControl(obj)
            figure,
            subplot(5,1,1)
            plot(obj.Tsol, obj.clfSol.V)
            title('V')
            subplot(5,1,2)
            plot(obj.Tsol, obj.clfSol.dV)
            title('dV')
            subplot(5,1,3)
            plot(obj.Tsol, obj.clfSol.delta)
            title('delta')
            h = subplot(5,1,4); hold(h, 'on')
            plot(obj.Tsol, obj.clfSol.cbfH(1,:), 'r')
            plot(obj.Tsol, obj.clfSol.cbfH(2,:), 'g')
            plot(obj.Tsol, zeros(size(obj.Tsol)), 'k')
            title('cbfH')
            subplot(5,1,5)
            plot(obj.Tsol, obj.clfSol.exitflag)
            title('exitflag')
            
            figure; hold on; grid on;
            plot(obj.Tsol,obj.usol)
            xlabel('time [s]')
            ylabel('leg length actuation [m/s^2')
            
            uTotal = trapz(obj.Tsol,abs(obj.usol(1,:))) + ...
                     trapz(obj.Tsol,abs(obj.usol(1,:)));
            nSteps = obj.stepCnt;
            uPerStep = uTotal/nSteps;
            annotation('textbox',[0.15 0.65 0.3 0.15],...
                       'String',{['uTotal = ',num2str(uTotal)],...
                                 ['nSteps = ',num2str(nSteps)],...
                                 ['uPerStep = ',num2str(uPerStep)]})

        end
        
        function animate(obj)
            T = obj.Tsol;
            X = obj.Xsol;
            animData = struct;
            [~, index] = unique(T);
            x = X(:,1); xdot = X(:,2);
            z = X(:,3); zdot = X(:,4);
            L1 = X(:,5); dL1 = X(:,6);
            L2 = X(:,7); dL2 = X(:,8);
            sL1 = X(:,9); dsL1 = X(:,10);
            sL2 = X(:,11); dsL2 = X(:,12);
            r1 = X(:, 13); dr1 = X(:,14);
            r2 = X(:, 15); dr2 = X(:,16);
            q1 = X(:, 17); dq1 = X(:,18);
            q2 = X(:, 19); dq2 = X(:,20);
            
            animData.t = {T(index)};
            animData.r1 = r1(index);
            animData.r2 = r2(index);
            animData.s1 = sL1(index);
            animData.s2 = sL2(index);
            animData.q1 = q1(index);
            animData.q2 = q2(index);
            animData.x = x(index);
            animData.z = z(index);
            
            % foot position
            animData.f1x = x(index) - animData.r1.*sin(animData.q1);
            animData.f2x = x(index) - animData.r2.*sin(animData.q2);
            
            animData.f1z = z(index) - animData.r1.*cos(animData.q1);
            animData.f2z = z(index) - animData.r2.*cos(animData.q2);
            
            obj.animData = animData; 
            scene = SLIPwalkingScene(animData, obj.terrain.x, obj.terrain.z);
            Player(scene)
        end
        
        function plotGRFDesired(obj)
            if obj.expectedDownstep
                h = obj.knownDownstepHeight;
                teval = linspace(0,1,100);
                figure; 
                % phase 1
                n = length(obj.downstepBeziersF.exp.phase1.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase1.grf_SSP{i},h);
                end
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

                subplot(1,3,1); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_st,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_sw,teval))

                % phase 2
                n = length(obj.downstepBeziersF.exp.phase2.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_SSP{i},h);
                end
                n = length(obj.downstepBeziersF.exp.phase2.grf_DSP_sw);
                bv_grf_DSP_sw = zeros(1,n);
                for i = 1:n
                    bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_DSP_sw{i},h);
                end
                n = length(obj.downstepBeziersF.exp.phase2.grf_DSP_st);
                bv_grf_DSP_st = zeros(1,n);
                for i = 1:n
                    bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_DSP_st{i},h);
                end

                subplot(1,3,2); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_st,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_sw,teval))

                % phase 3
                n = length(obj.downstepBeziersF.exp.phase3.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase3.grf_SSP{i},h);
                end

                subplot(1,3,3); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
            else
                %%% UNEXPECTED DOWNSTEP
                h = obj.knownDownstepHeight;
                teval = linspace(0,1,100);
                figure; 
                % phase 1
                n = length(obj.downstepBeziersF.unexp.phase1.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_SSP{i},h);
                end
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

                subplot(1,3,1); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_st,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_sw,teval))

                % phase 2
                n = length(obj.downstepBeziersF.unexp.phase2.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_SSP{i},h);
                end
                n = length(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw);
                bv_grf_DSP_sw = zeros(1,n);
                for i = 1:n
                    bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw{i},h);
                end
                n = length(obj.downstepBeziersF.unexp.phase2.grf_DSP_st);
                bv_grf_DSP_st = zeros(1,n);
                for i = 1:n
                    bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_st{i},h);
                end

                subplot(1,3,2); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_st,teval))
                plot(obj.TS+teval*obj.TD,bezier2(bv_grf_DSP_sw,teval))

                % phase 3
                n = length(obj.downstepBeziersF.unexp.phase3.grf_SSP);
                bv_grf_SSP = zeros(1,n);
                for i = 1:n
                    bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase3.grf_SSP{i},h);
                end

                subplot(1,3,3); hold on; grid on;
                plot(teval*obj.TS,bezier2(bv_grf_SSP,teval))
            end
            
        end
        
        function plotZDesired(obj)
            if obj.expectedDownstep
                h = obj.knownDownstepHeight;
                teval = linspace(0,1,100);
                figure; hold on; grid on;
                % phase 1
                n = length(obj.downstepBeziersZ.exp.phase1.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase1.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)

                % phase 2
                n = length(obj.downstepBeziersZ.exp.phase2.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase2.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)+(obj.TS+obj.TD)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)

                % phase 3
                n = length(obj.downstepBeziersZ.exp.phase3.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase3.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)+(obj.TS+obj.TD)+(obj.TS/2)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)
                
                % nominal
                plot(teval*(obj.TS+obj.TD)-obj.TS/2,bezier2(obj.nominalBeziers.bv_zcom,teval))
                plot(teval*(obj.TS+obj.TD)+obj.TS/2+obj.TD,bezier2(obj.nominalBeziers.bv_zcom,teval))
            else
                h = obj.knownDownstepHeight;
                teval = linspace(0,1,100);
                figure; hold on; grid on;
                % phase 1
                n = length(obj.downstepBeziersZ.unexp.phase1.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase1.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)

                % phase 2
                n = length(obj.downstepBeziersZ.unexp.phase2.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase2.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)+(obj.TS+obj.TD)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)

                % phase 3
                n = length(obj.downstepBeziersZ.unexp.phase3.zcom);
                bv_zcom = zeros(1,n);
                for i = 1:n
                    bv_zcom(i) = polyval(obj.downstepBeziersZ.unexp.phase3.zcom{i},h);
                end
                plot((obj.TS/2+obj.TD)+(obj.TS+obj.TD)+(obj.TS/2)*teval,bezier2(bv_zcom,teval)-obj.deltaNominal)
                
                % nominal
                plot(teval*(obj.TS+obj.TD)-obj.TS/2,bezier2(obj.nominalBeziers.bv_zcom,teval))
                plot(teval*(obj.TS+obj.TD)+obj.TS/2+obj.TD,bezier2(obj.nominalBeziers.bv_zcom,teval))
            end
        end
        
        function plotTransition(obj)
            phase = 'phase1';
            h = obj.zsw2f;

            if obj.stepTime > obj.TS+obj.TD
                t_norm = 1;
            else
                t_norm = obj.stepTime/(obj.TS+obj.TD);
            end
            
            timeMax = obj.stepTimeVLO + obj.TD;
            if obj.stepTimeVLO > timeMax
                t_norm_VLO = 1;
            else 
                t_norm_VLO = obj.stepTimeVLO/timeMax;
            end

            % get bezier polynomials from interpolation
            n = length(obj.downstepBeziersZ.unexp.phase1.zcom);
            bv_zcom = zeros(1,n);
            for i = 1:n
                bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.phase1.zcom{i},h);
            end
            
            teval = linspace(0,1,100);
            
            figure; hold on; grid on;
            plot(teval,bezier2(bv_zcom,teval) - obj.deltaNominal)
            plot(t_norm_VLO,bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal,'ro')
            plot(teval-0.4,bezier2(obj.nominalBeziers.bv_zcom,teval))
            plot(t_norm-0.4,bezier2(obj.nominalBeziers.bv_zcom,t_norm),'ro')
            legend('downstep','nominal')

        end
        
        function plotTransition2(obj)
            figure; 
            subplot(1,3,1); hold on; grid on;
            plot(obj.zcomNominalLog(:,1),obj.zcomNominalLog(:,2),'r')
            plot(obj.zcomDownstepLog(:,1),obj.zcomDownstepLog(:,2),'b')
            legend('nominal','downstep')
            title('zcom desired')
            
            subplot(1,3,2); hold on; grid on;
            plot(obj.zcomNominalLog(:,1),obj.zcomNominalLog(:,3),'r')
            plot(obj.zcomDownstepLog(:,1),obj.zcomDownstepLog(:,3),'b')
            legend('nominal h','downstep h')
            title('zsw detection')
            
            subplot(1,3,3); hold on; grid on;
            plot(obj.zcomNominalLog(:,1),obj.zcomNominalLog(:,4),'r')
            plot(obj.zcomDownstepLog(:,1),obj.zcomDownstepLog(:,4),'b')
            legend('time_norm','time_norm_VLO')
            title('time_norm')
        end
        
        function plotComparisonToHuman(obj)
            X = obj.Xsol;
            t = obj.Tsol;
            
            x = X(:,1);     dx = X(:,2);
            z = X(:,3);     dz = X(:,4);
            L1 = X(:,5);    dL1 = X(:,6); %original left leg
            L2 = X(:,7);    dL2 = X(:,8);
            sL1 = X(:,9);   dsL1 = X(:,10);
            sL2 = X(:,11);  dsL2 = X(:,12);
            r1 = X(:, 13);  dr1 = X(:, 14);
            r2 = X(:, 15);  dr2 = X(:, 16);
            q1 = X(:, 17);  dq1 = X(:, 18);
            q2 = X(:, 19);  dq2 = X(:, 20);
            
            % load and create human data
            load('data/beziersRaw/allBeziersHuman.mat')
            dxcom_human = []; time_human = [];
            dzcom_human = []; 
            xcom_human = [];
            zcom_human = [];
            
            teval = linspace(0,1,100);
            for i = {'phase1','phase2','phase3'}
                bv_dxcom = allBeziers.(obj.exp).(i{:}).bv_dxcom;
                dxcom_human = [dxcom_human bezier2(bv_dxcom,teval)];
                
                bv_dzcom = allBeziers.(obj.exp).(i{:}).bv_dzcom;
                dzcom_human = [dzcom_human bezier2(bv_dzcom,teval)];
                
                bv_xcom = allBeziers.(obj.exp).(i{:}).bv_xcom;
                xcom_human = [xcom_human bezier2(bv_xcom,teval)];
                
                bv_zcom = allBeziers.(obj.exp).(i{:}).bv_zcom;
                zcom_human = [zcom_human bezier2(bv_zcom,teval)];
                
                tactual = linspace(0,allBeziers.(obj.exp).(i{:}).timeMax);
                try
                    tactual = tactual + time_human(end);
                end
                time_human = [time_human tactual];
            end
            lw = 3;
            fontSize = 15;
            
            %%% dx, z, dz, GRF
            figure
            subplot(4,1,1); hold on; grid on;
            plot(t-obj.tLastVLO,dx,'r','LineWidth',lw)
            plot(time_human,dxcom_human,'b','LineWidth',lw)
            legend('aSLIP','human')
            title('t-dx');
            set(gca,'FontSize',fontSize)
            
            subplot(4,1,2); hold on; grid on;
            plot(t-obj.tLastVLO,z,'r','LineWidth',lw)
            plot(time_human,zcom_human,'b','LineWidth',lw)
            legend('aSLIP','human')
            title('t-z');
            set(gca,'FontSize',fontSize)
            
            subplot(4,1,3); hold on; grid on;
            plot(t-obj.tLastVLO,dz,'r','LineWidth',lw)
            plot(time_human,dzcom_human,'b','LineWidth',lw)
            legend('aSLIP','human')
            title('t-dz');
            set(gca,'FontSize',fontSize)
            
            subplot(4,1,4); hold on; grid on;
            plot(t-obj.tLastVLO, obj.GRFsol(1,:), 'b','LineWidth',lw)
            plot(obj.GRFsbound(:,1)-obj.tLastVLO,obj.GRFsbound(:,3),'r','LineWidth',lw)
            plot(t-obj.tLastVLO, obj.GRFsol(2,:), 'b','LineWidth',lw) 
            plot(obj.GRFnsbound(:,1)-obj.tLastVLO,obj.GRFnsbound(:,3),'r','LineWidth',lw)
            title('F_z (N)')
            legend('aSLIP','human');
            set(gca,'FontSize',fontSize)
            
            %%% stepsizes
            load('data/human/Lstep_struct.mat')
            LstepsHuman = Lstep_struct.(obj.exp).steps_mean;
            LstepsaSLIP = obj.stepLengthSequence(obj.knownDownstepStep:obj.knownDownstepStep+3);
            figure; hold on; grid on;
            p = plot(1:4,LstepsaSLIP,'ro');
            p.MarkerSize = 10;
            p.MarkerFaceColor = 'r';
            p = plot(1:4,LstepsHuman,'bo');
            p.MarkerSize = 10;
            p.MarkerFaceColor = 'b';
            ylabel('Stepsize [m]')
            xlim([0 5])
            xticks([1 2 3 4])
            xticklabels({'nominal','downstep','overstep','upstep'})
            title(obj.exp)
            legend('aSLIP','human')
            set(gca,'FontSize',fontSize)
        end
        
        function [Wx] = calculationW(obj)
            %%% calculate w %% model error %%
            Wx = [];
    
            pVec = obj.DiscreteX.p;
            vVec = obj.DiscreteX.v;
            uVec = obj.stepLengthSequence;
            
            A = obj.LIP.Ahat0;
            B = obj.LIP.Bhat0;
            
            X = [pVec;vVec];
            Ux = uVec;
            Xlip = obj.LIP.XLIPlist; %obj.log.LIP.Xdes; %%LIPx.X3LIPlist;
            
            for i = 1:length(X)-1
                wx = X(:, i+1) -  A*X(:,i) - B*Ux(i+1);
                Wx = [Wx, wx];
            end
            
            boundW = max(abs(Wx),[], 2);
            w0min = -boundW(1); w1min = -boundW(2);
            w0max = boundW(1); w1max = boundW(2);
            
            WxSet = Polyhedron([w0min, w1min; ...
                w0min, w1max; ...
                w0max, w1min; ...
                w0max, w1max;...
                ]);
            WxSet.computeHRep;
            figure, box on, hold on; 
            plot(Wx(1,:), Wx(2,:));
            WxSet.plot('color','b','alpha',0.1,'linewidth',1,'linestyle','--');
        end
    end
    
    methods %%% control
        function u = backStepping(obj, t)
            r = obj.polar.rs;
            dr = obj.polar.drs;
            s = obj.polar.sLs;
            ds = obj.polar.dsLs;
            qs = obj.polar.qs;
            dqs = obj.polar.dqs;
            Ls = obj.polar.Ls;
            zdes = interp1(obj.desiredbehavior.t, obj.desiredbehavior.zdes, t);
            dzdes = interp1(obj.desiredbehavior.t, obj.desiredbehavior.dzdes, t);
            ddzdes = interp1(obj.desiredbehavior.t, obj.desiredbehavior.ddzdes, t);
            Fs = obj.springForce(Ls, s, ds);
            %%% eta1 = z - zdes;
            %%% eta2 = dz - dzdes;
            %%% ddz = (obj.polar.Fs*cos(obj.polar.qs) + obj.polar.Fns*cos(obj.polar.qns))/obj.m - obj.g;
            %%% ddeta = ddz - ddzdes =  -g + 1/obj.m*F*cos(qs) - ddzdes = ddeta_FB_IO =  -kp*eta -kd*deta
            %%% ddr_stance = (Fstance + Fnonstance*cos(q_stance - q_nonstance))/obj.mass - obj.g*cos(q_stance) + r_stance*dq_stance^2;
            %%%% ddr_stance = Fstance/obj.mass - obj.g*cos(q_stance) + r_stance*dq_stance^2;
            %%% dF = K*ds + D*dds = K*ds + D*(u - ddr) = D*u + K*ds -
            %%% D*(-g*cos(qstance) + r_stance*dq_stance^2 + 1/obj.m*F) = f2 + g2*u;
            eta = [obj.polar.z  - zdes;
                   obj.polar.dz - dzdes];
               
            %%% deta = f1 + g1*F;
            f1 = [obj.polar.dz - dzdes; -obj.g - ddzdes];
            g1 = [0; 1/obj.m*cos(qs)];
            
            %%% F = f2 + g2*u;
            f2 = obj.K(Ls,obj.Kparam)*ds - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + r*dqs^2 + 1/obj.m*Fs);
            g2 = obj.D(Ls,obj.Dparam); 
            
            %V_eta = 1/2*eta^T*eta;
            partial_V_eta = transpose(eta);
            
            k = 100; % K > min(eig([0,1;kp, kd]))
            pz = 1;
            Fbar_fbIO = obj.m/cos(qs)*(obj.g + ddzdes - 1/(obj.epsilon^2)*obj.Kp*eta(1) - 1/obj.epsilon*obj.Kd*eta(2));
            u = (-partial_V_eta*g1 - k*(Fs - Fbar_fbIO))/pz + [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd]*(f1 + g1*Fs);
            u = (u-f2)/g2;
            
            u = [u;0];
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = QPbackSteppingCLFCBF(obj, t)
            if obj.NcontactLegs == 1
                [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = obj.SSPbackSteppingCLFCBF(t);
%                 [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = obj.OutputPD_QP(t);
            else
                [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = obj.DSPUnitedBackSteppingCLFCBF(t);
                obj.newAvoid = 0; %%% temp for reg regions to avoid stepping (stairs)
            end
            %%% u = [us; uns]
            if obj.stanceLegNum == 1 %%% right stance 
                us = u(1); 
                uns = u(2); 
                u = [uns; us];
            end
           % u = obj.boundU(u);
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = SSPbackSteppingCLFCBF(obj, t)
            persistent usol_prev
            if isempty(usol_prev)
                usol_prev = zeros(2,1);
            end
            
            rs = obj.polar.rs;
            drs = obj.polar.drs;
            s = obj.polar.sLs;
            ds = obj.polar.dsLs;
            qs = obj.polar.qs;
            dqs = obj.polar.dqs;
            Ls = obj.polar.Ls;
            dLs = obj.polar.dLs;
            
            % Compute desired zcom, either constant (from filtered ground
            % profile) or from bezier splines
            [zdes, dzdes, ddzdes] = obj.getDesiredZ(t);
            zdesStruct.zdes = zdes;
            zdesStruct.dzdes = dzdes;
            zdesStruct.ddzdes = ddzdes;
            obj.zdesPrev = zdesStruct;
            
            % Compute desired GRF from splines
            [Fsdes, dFsdes] = obj.getDesiredF(t);
            FdesStruct.Fsdes = Fsdes;
            FdesStruct.dFsdes = dFsdes;
            obj.FdesPrev = FdesStruct;
            
            dddzdes = 0; 
            Fs = obj.springForce(Ls, s, ds);
            %%% eta1 = z - zdes;
            %%% eta2 = dz - dzdes;
            %%% ddz = (obj.polar.Fs*cos(obj.polar.qs) + obj.polar.Fns*cos(obj.polar.qns))/obj.m - obj.g;
            %%% ddeta = ddz - ddzdes =  -g + 1/obj.m*F*cos(qs) - ddzdes = ddeta_FB_IO =  -kp*eta -kd*deta
            %%% ddr_stance = (Fstance + Fnonstance*cos(q_stance - q_nonstance))/obj.mass - obj.g*cos(q_stance) + r_stance*dq_stance^2;
            %%%% ddr_stance = Fstance/obj.mass - obj.g*cos(q_stance) + r_stance*dq_stance^2;
            %%% dF = K*ds + D*dds = K*ds + D*(u - ddr) = D*u + K*ds -
            %%% D*(-g*cos(qstance) + r_stance*dq_stance^2 + 1/obj.m*F) = f2 + g2*u;
            eta = [obj.polar.z - zdes;
                   obj.polar.dz - dzdes];
            %%% deta = f1 + g1*F;
            %%% F = f2 + g2*u;
            f1 = [obj.polar.dz - dzdes; 
                  -obj.g - ddzdes];
            g1 = [0; 
                  1/obj.m*cos(qs)];
            deta = f1 + g1*Fs; 
            
            % old K
            ddrs = -obj.g*cos(qs) + rs*dqs^2 + 1/obj.m*Fs;
            % g2 = obj.D(Ls,obj.Dparam); 
            % f2 = obj.K(Ls,obj.Kparam)*ds - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + rs*dqs^2 + 1/obj.m*Fs);
            % new K(L)
            gs = obj.D(Ls,obj.Dparam); 
            fs = obj.K(Ls,obj.Kparam)*dLs*s + obj.K(Ls,obj.Kparam)*ds + obj.D(Ls,obj.Dparam)*dLs*ds - ...
                                - obj.D(Ls,obj.Dparam)*ddrs;
           
            k = 10; % K > min(eig([0,1;kp, kd]))
            pz = 1; 
            KIO =  [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd];
            Acl = [0, 1; KIO]; 
            Q = eye(2); 
            P = lyap(Acl, Q); 
             V_eta = 1/2*transpose(eta)*P*eta;
            partial_V_eta = transpose(eta)*P;
            [~,eigP] = eig(P);
            [~,eigQ] = eig(Q);

            % obj.gamma = min([eigQ(1,1), eigQ(2,2)])/max([eigP(1,1), eigP(2,2)]); 
            Fbar_fbIO = obj.m/cos(qs)*(obj.g + ddzdes + KIO*eta);
            dFbar_fbIO = obj.m/cos(qs)*(dddzdes + KIO*deta) + obj.m*(obj.g + ddzdes + KIO*eta)*sec(qs)*tan(qs)*dqs; 
            
            udes =( -partial_V_eta*g1 - obj.Kbackstepping*(Fs - Fbar_fbIO))/pz + [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd]*(f1 + g1*Fs);
            udes = (udes - fs)/gs; 
            
            %%%%%%%%%%%%%%% CLF constraint construction %%%%%%%%%%%%%%
            z = Fs - Fbar_fbIO;
            V = V_eta + 1/2*pz*z^2; 
            dVeta = partial_V_eta*deta; 
            %%%% Vdot := LvF + LvG*u = dVeta + pz*z*dz 
            %%%% dz = dFs - dFbar_fbIO = f2 + g2*u - dFbar_fbIO; 
            %%%% dV < -gamma*V + delta; LvF + LvG*u <  -gamma*V + delta;  Aclf*u < bclf
            LvF = dVeta + pz*z*(fs - dFbar_fbIO); 
            LvG = pz*z*gs; 
            Aclf =  [LvG, -1e+3];
            Bclf = -obj.gamma*V - LvF; 
            
            %%%%%%%%%%%%%%%%%%%% CBF constraint construction %%%%%%%%%%%%
            deltaF = obj.deltaF;
            if obj.useHumanF
                if obj.useDecreasingRelaxation
                    c_relaxs = [0.8 0.8 0.8 linspace(0.8,obj.c_relax_SSP,obj.stepsToTrueDesired)];
                    index = length(c_relaxs);
                    if obj.stepCnt < length(c_relaxs)
                        index = obj.stepCnt;
                    end
                    c = c_relaxs(index);
                else
                    c = obj.c_relax_SSP;
                end
                
                if obj.isDownstep
                    c = obj.c_relax_SSP_downstep;
                end
                
                hs = (c*Fsdes + deltaF)^2 - (Fs - Fsdes)^2;
                Acbf = [2*(Fs-Fsdes)*gs, 0];
                Bcbf = c^2*2*Fsdes*dFsdes +  2*c*deltaF*dFsdes- 2*(Fs - Fsdes)*(fs - dFsdes) + obj.beta*hs;
                
                %%%% log the admissible force set: 
                obj.GRFsbound = [obj.GRFsbound; [t, (1-c)*Fsdes - deltaF, Fsdes, (1+c)*Fsdes + deltaF]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, (1-c)*Fsdes - deltaF, Fsdes, (1+c)*Fsdes + deltaF]]; 
            else
                Fsdes = 0;
                hs = Fs - obj.Fmin; 
                % dh = f2 + g2*u > - beta*h, 
                % Acbf*u < bcbf
                Acbf = [-gs, 0];
                Bcbf = fs + obj.beta*hs;
                
                %%%% log the admissible force set: 
                obj.GRFsbound = [obj.GRFsbound; [t, 0, 0, 0]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0, 0]]; 
            end
            
            A = [Aclf; Acbf]; 
            ubA = [Bclf; Bcbf];
            lbA = -inf*ones(size(ubA));
            lb = [-obj.ddLmax; -inf]; 
            ub = [obj.ddLmax; inf];
            
            %%%%%%%%%%%% QP construction %%%%%%%%%%%%%%%%%%%%%%%%
             %%%% min |u - udes|^2 + p*delta^2
             %%%% min |u|^2 + |LvF + LvG*u|^2 + p*delta^2

            p = 1e+5;
            H = [1,0;0,p]; 
            G = [0;0];
            nWSR = 10000;
            
            options = optimoptions('quadprog','Display','off');
            [usol,fval,exitflag, numiter] = quadprog(H,G, A, ubA, [], [], lb, ub,[],options);
            if exitflag == -2
                disp("--- SSP QP ERROR!!! ---")
                usol = usol_prev
%                 usol = zeros(2,1);
            end
            
            delta = usol(2); 
            u_nonstance = obj.swingLegControl(t); 
            u = [usol(1); u_nonstance];
            
            dV = LvF + LvG*usol(1); 
            cbfH = Acbf*usol - Bcbf;
            cbfH = [cbfH; 0;]; 
            usol_prev = usol;
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = DSPUnitedBackSteppingCLFCBF(obj, t)
            persistent passedDSP1 DSP1min DSP1max
            if isempty(passedDSP1)
                passedDSP1 = false;
            end
            
            persistent passedDSP2 DSP2min DSP2max
            if isempty(passedDSP2)
                passedDSP2 = false;
            end
            
            %%% combine the input, using CBF for stance force as well. 
            rs = obj.polar.rs;            rns = obj.polar.rns;
            drs = obj.polar.drs;          drns = obj.polar.drns;
            ss = obj.polar.sLs;           sns = obj.polar.sLns;
            dss = obj.polar.dsLs;         dsns = obj.polar.dsLns;
            qs = obj.polar.qs;            qns = obj.polar.qns;
            dqs = obj.polar.dqs;          dqns = obj.polar.dqns;
            Ls = obj.polar.Ls;            Lns = obj.polar.Lns;
            dLs = obj.polar.dLs;          dLns = obj.polar.dLns;

            % Compute desired zcom, either constant (from filtered ground
            % profile) or from bezier splines
            [zdes, dzdes, ddzdes] = obj.getDesiredZ(t);
            zdesStruct.zdes = zdes;
            zdesStruct.dzdes = dzdes;
            zdesStruct.ddzdes = ddzdes;
            obj.zdesPrev = zdesStruct;
            
            % Compute desired GRF from splines
            [Fdes, dFdes] = obj.getDesiredF(t);
            FdesStruct.Fsdes = Fdes(2);
            FdesStruct.dFsdes = dFdes(2);
            FdesStruct.Fnsdes = Fdes(1);
            FdesStruct.dFnsdes = dFdes(1);
            obj.FdesPrev = FdesStruct;
            
            % Old for GRF going to zero
            tInDomain = t - obj.TimeStamp(end); 
            tInDomain(tInDomain<0) = 0; 
            Fsdes = obj.FliftOff*interp1(obj.desiredGRF.t, obj.desiredGRF.Fmax, tInDomain);
            dFsdes = obj.FliftOff*interp1(obj.desiredGRF.t, obj.desiredGRF.dFmax, tInDomain);

            dddzdes = 0; 
            Fs = obj.springForce(Ls, ss, dss);
            Fns = obj.springForce(Lns, sns, dsns);
            
            %%%%%%%%%%%%%%%% united Fs and Fns both as inputs
            %%% eta1 = z - zdes; eta2 = dz - dzdes;
            Fsum = Fs*cos(qs) + Fns*cos(qns);
            %%% ddz = Fsum/obj.m - obj.g;
            %%% ddeta = ddz - ddzdes =  -g + Fsum/obj.m - ddzdes = ddeta_FB_IO =  -kp*eta -kd*deta
            eta = [obj.polar.z - zdes;
                   obj.polar.dz - dzdes];
            f0 = [obj.polar.dz - dzdes; -obj.g - ddzdes];
            g0 = [0; 1/obj.m];
            deta = f0 + g0*Fsum; 
            
            ddrs = (Fs + Fns*cos(qs - qns))/obj.m - obj.g*cos(qs) + rs*dqs^2;
            ddrns = (Fns + Fs*cos(qs - qns))/obj.m - obj.g*cos(qns) + rns*dqns^2;
            %%% dFs = K*dsLs + D*ddsLs = K*dsLs + D*(u1 - ddrs) = D*u1 + K*dsLs - D*ddrs = fs + gs*u1;
            %%% dFns = K*dsLns + D*ddsLns = K*dsLns + D*(u2 - ddrns) = D*u2 + K*dsLns - D*ddrns = fns + gns*u2;
            % old K
            % fs = obj.K(Ls,obj.Kparam)*dsLs - obj.D(Ls,obj.Dparam)*ddrs;
            % gs = obj.D(Ls,obj.Dparam); 
            % fns = obj.K(Lns,obj.Kparam)*dsLns - obj.D(Lns,obj.Dparam)*ddrns;
            % gns = obj.D(Lns,obj.Dparam); 
            % new K(L)
            gs  = obj.D(Ls,obj.Dparam); 
            fs  = obj.K(Ls,obj.Kparam)*dLs*ss + obj.K(Ls,obj.Kparam)*dss + obj.D(Ls,obj.Dparam)*dLs*dss - ...
                                - obj.D(Ls,obj.Dparam)*ddrs;
            gns = obj.D(Lns,obj.Dparam); 
            fns = obj.K(Lns,obj.Kparam)*dLns*sns + obj.K(Lns,obj.Kparam)*dsns + obj.D(Lns,obj.Dparam)*dLns*dsns - ...
                                - obj.D(Lns,obj.Dparam)*ddrns;
                            

            
            f_sum = -Fs*sin(qs)*dqs - Fns*sin(qns)*dqns + cos(qs)*fs + cos(qns)*fns;
            g_sum = 1;  %%%% u_sum = cos(qs)*gs*us + cos(qns)*gns*uns 
            k = 20; %200; 
            us_des = 1/gs*(-fs - k*(Fs - Fsdes) + dFsdes);
            %%% control the stance Force %%%%
            V_eta = 1/2*transpose(eta)*eta;
            partial_V_eta = transpose(eta);
            
            % K > min(eig([0,1;kp, kd]))
            pz = 1; %%%%%% coef on the cost function 
            KIO =  [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd];
            
            Fsumbar_fbIO = obj.m*(obj.g + ddzdes + KIO*eta);
            dFsumbar_fbIO = obj.m*(dddzdes + KIO*deta ); 
            
            u_sum_des =( -partial_V_eta*g0 - obj.Kbackstepping*(Fsum - Fsumbar_fbIO))/pz + [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd]*deta;
            u_sum_des = (u_sum_des - f_sum)/g_sum; 
            %%%%%%%%%%%%%%% CLF constraint construction %%%%%%%%%%%%%%
            z = Fsum - Fsumbar_fbIO;
            V = V_eta + 1/2*pz*z^2; 
            dVeta = partial_V_eta*deta; 
            %%%% Vdot := LvF + LvG*u_sum = dVeta + pz*z*dz 
            %%%% dz = dFsum - dFsumbar_fbIO = f_sum + g_sum*u_sum - dFsumbar_fbIO; 
            %%%% dV < -gamma*V + delta; LvF + LvG*u_sum <  -gamma*V + delta;  Aclf*u < bclf
            %%%% u_sum = cos(qs)*gs*us + cos(qns)*gns*uns

            LvF = dVeta + pz*z*(f_sum - dFsumbar_fbIO); 
            LvG = pz*z*g_sum; 
            Aclf =  [LvG*cos(qs)*gs, LvG*cos(qns)*gns, -1e+3];
            Bclf = -obj.gamma*V - LvF; 
            
            
            %%%%%%%%%%%%%%%%%%%% CBF constraint construction %%%%%%%%%%%%
            deltaF = obj.deltaF; 
            if obj.useHumanF
                if obj.useDecreasingRelaxation
                    c_relaxs = [0.8 0.8 0.8 linspace(0.8,obj.c_relax_DSP,obj.stepsToTrueDesired)];
                    index = length(c_relaxs);
                    if obj.stepCnt < length(c_relaxs)
                        index = obj.stepCnt;
                    end
                    c = c_relaxs(index);
                else
                    c = obj.c_relax_DSP;
                end
                
                if obj.isDownstep 
                    c = obj.c_relax_DSP_downstep;
                end
                
                hs = (c*FdesStruct.Fsdes + deltaF)^2 - (Fs - FdesStruct.Fsdes)^2;
                Acbf_s = [2*(Fs-FdesStruct.Fsdes)*gs, 0, 0];
                Bcbf_s = c^2*2*FdesStruct.Fsdes*FdesStruct.dFsdes + ...
                         2*c*deltaF*FdesStruct.dFsdes - ...
                         2*(Fs - FdesStruct.Fsdes)*(fs - FdesStruct.dFsdes) + ...
                         obj.beta*hs;
                
                obj.GRFsbound  = [obj.GRFsbound; [t, (1-c)*FdesStruct.Fsdes - deltaF, FdesStruct.Fsdes, (1+c)*FdesStruct.Fsdes + deltaF]]; 
                
%                 hns = Fns; 
%                 Acbf_ns = [0, -gns, 0];
%                 Bcbf_ns = fns + obj.beta*hns;
                            
                            
%                 if obj.downstepStep == 1
%                     if ~passedDSP1 
%                         % should be Bezier with $h$ 
%                         DSP1min = obj.stepTime;
%                         DSP1max = DSP1min + obj.TD;
%                         passedDSP1 = true;
%                     end
%                     tTmp = clamp(obj.stepTime,DSP1min,DSP1max);
%                     t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min);
%                 else
                tTmp = clamp(obj.stepTime,obj.TS,obj.TS+obj.TD);
                t_norm = (tTmp - obj.TS) / (obj.TD);
%                 end
                deltaF = deltaF + (1-t_norm)*200;
                
                hns = (c*FdesStruct.Fnsdes + deltaF)^2 - (Fns - FdesStruct.Fnsdes)^2;
                Acbf_ns = [0, 2*(Fns - FdesStruct.Fnsdes)*gns, 0];
                Bcbf_ns = c^2*2*FdesStruct.Fnsdes*FdesStruct.dFnsdes + ...
                          2*c*deltaF*FdesStruct.dFnsdes - ...
                          2*(Fns - FdesStruct.Fnsdes)*(fns - FdesStruct.Fnsdes) + ...
                          obj.beta*hns;
                      
                obj.GRFnsbound = [obj.GRFnsbound; [t, (1-c)*FdesStruct.Fnsdes - deltaF, FdesStruct.Fnsdes, (1+c)*FdesStruct.Fnsdes + deltaF]];
%                 obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0]]; 
            else
                hs = (obj.c_relax_DSP*Fsdes + deltaF)^2 - (Fs - Fsdes)^2; 
                Acbf_s = [2*(Fs-Fsdes)*gs, 0, 0];
                Bcbf_s = obj.c_relax_DSP^2*2*Fsdes*dFsdes + ...
                         2*obj.c_relax_DSP*deltaF*dFsdes - ...
                         2*(Fs - Fsdes)*(fs- dFsdes) + ...
                         obj.beta*hs;
                
                hns = Fns;  % obj.Fmin;
                % dh = fns + gns*uns > - beta*h, Acbf*u < bcbf
                Acbf_ns = [0, -gns, 0];
                Bcbf_ns = fns + obj.beta*hns;
                
                %%%% log the admissible force set: 
                obj.GRFsbound  = [obj.GRFsbound; [t, (1-obj.c_relax_DSP)*Fsdes - deltaF, Fsdes (1+obj.c_relax_DSP)*Fsdes + deltaF]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0, 0]]; 
            end
            

            A = [Aclf; Acbf_s;  Acbf_ns]; 
            ubA = [Bclf; Bcbf_s; Bcbf_ns]; 

            lbA = -inf*ones(size(ubA));
            lb = [-obj.ddLmax;-obj.ddLmax; -inf]; 
            ub = [obj.ddLmax; obj.ddLmax;inf];
            %%%%%%%%%%%% QP construction %%%%%%%%%%%%%%%%%%%%%%%%
            %%%% u_sum = cos(qs)*gs*us + cos(qns)*gns*uns = as*us +
            %%%% ans*uns; 
            a1 = cos(qs)*gs; a2 = cos(qns)*gns;
            %%%% min |u_sum - u_sum_des|^2 + p*delta^2
            p = 1e+4;
            % Hu = [a1^2, a1*a2; a1*a2, a2^2];
            % H = blkdiag(Hu, p);
            % G = [-2*u_sum_des*a1; -2*u_sum_des*a2;0];
            % p = 1;
            H = blkdiag(eye(2), p);
            G = zeros(3,1); 
            nWSR = 10000;
            %              options = qpOASES_options('maxIter',nWSR,'printLevel',1);
            %              [sol,fval,exitflag, numiter] = qpOASES(H, G, A, lb, ub, lbA,ubA,options);
            options = optimoptions('quadprog','Display','iter', 'MaxIterations', 20000);
            [sol,fval,exitflag, numiter] = quadprog(H, G, A, ubA, [], [], lb, ub,[],options);
            if exitflag ~= -2
                delta = sol(3);
                u = [sol(1); sol(2)];
                u_sum = a1*u(1) + a2*u(2);
                dV = LvF + LvG*u_sum;
                cbfHns = Acbf_ns*sol - Bcbf_ns;
                cbfHs = Acbf_s*sol - Bcbf_s;
                cbfH = [cbfHs; cbfHns];
            else
                sol = zeros(3,1); 
                delta = sol(3);
                u = [sol(1); sol(2)];
                u_sum = a1*u(1) + a2*u(2);
                dV = LvF + LvG*u_sum;
                cbfHns = Acbf_ns*sol - Bcbf_ns;
                cbfHs = Acbf_s*sol - Bcbf_s;
                cbfH = [cbfHs; cbfHns];
            end
            
        end
        
        function u_nonstance = swingLegControl(obj, t)
            tInDomain = t - obj.TimeStamp(end);
            swingX = obj.polar.x - obj.polar.Lns*sin(obj.polar.qns); 
            swingdX = obj.polar.dx - obj.polar.dLns*sin(obj.polar.qns) - obj.polar.Lns*cos(obj.polar.qns)*obj.polar.dqns; 
            
            swingZ = obj.polar.z - obj.polar.Lns*cos(obj.polar.qns);
            swingdZ = obj.polar.dz - obj.polar.dLns*cos(obj.polar.qns) + obj.polar.Lns*sin(obj.polar.qns)*obj.polar.dqns;
            %%% ddZ = obj.polar.ddz -  obj.polar.ddLns*cos(obj.polar.qns)
            %%% + obj.polar.dLns*sin(obj.polar.qns)*obj.polar.dqns  + ...
            %%%% obj.polar.dLns*sin(obj.polar.qns)*obj.polar.dqns + obj.polar.Lns*cos(obj.polar.qns)*obj.polar.dqns^2 + ...
            %%%% obj.polar.Lns*sin(obj.polar.qns)*obj.polar.ddqns;
            ddz = (obj.polar.Fs*cos(obj.polar.qs) + obj.polar.Fns*cos(obj.polar.qns))/obj.m - obj.g;
            swingX = clamp(swingX, obj.terrain.x(1), obj.terrain.x(end)); 
            
            switch obj.downstepStep
%                 case 100
%                     swingZdes = interp1(obj.swingZbehaviorOS.t, obj.swingZbehaviorOS.z, tInDomain);
%                     swingdZdes = interp1(obj.swingZbehaviorOS.t, obj.swingZbehaviorOS.dz, tInDomain);
                case 3
                    swingZdes = interp1(obj.swingZbehaviorUS.t, obj.swingZbehaviorUS.z, tInDomain);
                    swingdZdes = interp1(obj.swingZbehaviorUS.t, obj.swingZbehaviorUS.dz, tInDomain);
                otherwise 
                    swingZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.z, tInDomain);
                    swingdZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.dz, tInDomain);
            end
            
%             swingZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.z, tInDomain);
%             swingdZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.dz, tInDomain);         
            terrainZ = interp1(obj.terrain.x, obj.terrain.swingZfilter, swingX);
            
            slope = interp1(obj.terrain.x, obj.terrain.theta, swingX); 
            dterrainZ = tan(slope)*swingdX; 
            
            swingZdes = swingZdes + terrainZ;
            swingdZdes = swingdZdes + dterrainZ;

            coef = 100;  %3; %100
            swingddz_des = [-1/(obj.epsilon^2)*obj.Kp, -1/obj.epsilon*obj.Kd]*coef*([swingZ - swingZdes; swingdZ - swingdZdes]);
            kp = 100; kd = 10; 
            swingddz_des = [-kp, -kd]*coef*([swingZ - swingZdes; swingdZ - swingdZdes]);
            u_nonstance = 1/cos(obj.polar.qns)*(-swingddz_des + ddz);% + obj.polar.dLns*sin(obj.polar.qns)*obj.polar.dqns  + ...
%             obj.polar.dLns*sin(obj.polar.qns)*obj.polar.dqns + obj.polar.Lns*cos(obj.polar.qns)*obj.polar.dqns^2 );
        end
        
        %%%%% TODO output level control on the swing foot  
        
        function u = boundU(obj, u)
            u1 = u(1); u2 = u(2); 
            u1(u(1)>obj.ddLmax) = obj.ddLmax; 
            u1(u(1)<-obj.ddLmax) = -obj.ddLmax; 
            u2(u(2) >obj.ddLmax) = obj.ddLmax; 
            u2(u(2) <-obj.ddLmax) = -obj.ddLmax; 
            u = [u1; u2]; 
        end 
        
        function [stepL, dstepL] = StepController(obj, t, x, x2f, z2f, dx)
            %%%% state based- LIP inpsired step size
            %%% project to slope coordinates
            %%%%
            % Either use theta from the ground, or detect it from the
            % current swing foot position. This we can only do after step 1
            % because the initial condition might screw up the first time
            % instance
            if obj.expectedDownstep
                if obj.stepCnt == obj.knownDownstepStep
                    theta = atan( obj.knownDownstepHeight/(obj.stepLengthSequence(end)/2));
                elseif obj.stepCnt == obj.knownDownstepStep + 1
                    theta = 0; %atan( -obj.knownDownstepHeight/(obj.stepLengthSequence(end-1)/2));
                else
                    theta = 0;
                end
            else
                if obj.downstepStep == 1
                    theta = atan( obj.zsw2f/(obj.stepLengthSequence(end)/2));
                elseif obj.downstepStep == 2
                    theta = 0; %atan(-obj.downstepHeightDetected/(obj.stepLengthSequence(end-1)/2));
                else
                    theta = 0;
                end
            end
            obj.LIP.theta = theta;        
            tliftOff = obj.TimeStamp(end);

            
            
            tNow = t - tliftOff;
            if isfield(obj.polar,'Fs')
                obj.LIP.LIPacc = (obj.polar.Fs*sin(obj.polar.qs) + obj.polar.Fns*sin(obj.polar.qns))/obj.m;
            end
            
            x2f = x2f*cos(theta);
            dx = dx*cos(theta);
            
            [obj.stepLdes, obj.dstepLdes] = obj.LIP.LIPbasedController(t, x, x2f, dx, obj.TS - tNow);
            obj.stepLdesOriginalLog = [obj.stepLdesOriginalLog; obj.stepLdes];
            
            %%% smoothing from the current swing foot position
            stepLengthPre = - obj.stepLengthSequence(end);
           
            cNow = interp1(obj.dumpVec, obj.smfVec, tNow)'; %cNow from 0 ->1
            dcNow = interp1(obj.dumpVec, obj.dsmfVec, tNow)'; %cNow from 0 ->1

            stepL = obj.stepLdes.*cNow + (1-cNow).*stepLengthPre;
            dstepL = obj.dstepLdes.*cNow + obj.stepLdes.*dcNow + (-dcNow).*stepLengthPre; %% important for swing Z control.
            
            %% offsets
            % human
            %   exp25: [0.71, 0.75]
            %   exp50: [0.71, 0.75]
            %   exp75: [0.71, 0.75]
            %   exp100: [0.71, 0.75] (with c = 0.40)
            %   unexp25: [0.71 0.75]
            %   unexp50: [0.71 0.75]
            %   unexp75: [0.71 0.75]
            %   unexp100: [0.71 0.75]
            
            % cassie
            %   exp25: [0.71, 0.75]
            %   exp50: 
            %   exp75: 
            %   exp100: 
            %   unexp25: 
            %   unexp50: 
            %   unexp75: 
            %   unexp100: 
            
%             if obj.downstepStep == 1
%                 obj.maxStepsize = 0.45; % for human unexpected 
%             elseif obj.downstepStep == 2
%                 obj.maxStepsize = Inf;
%             else
%                 obj.maxStepsize = Inf;
%             end
            stepL = clamp(stepL,-obj.maxStepsize,obj.maxStepsize);
        end
    end 
    
    methods  %%% gen desired  
        function genDesiredbehaviorUnexpDownstep(obj, downstep)
            obj.knownDownstepHeight = -downstep;
            obj.terrain.genUnexpDownstep(downstep);
            x = obj.terrain.x;
            zdes = obj.z0*ones(size(obj.terrain.dz)); 
            dzdxdes = zeros(size(obj.terrain.dz));
            ddzdxdes = zeros(size(obj.terrain.dz));
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorExpDownstep(obj, downstep)
            obj.knownDownstepHeight = downstep;
            obj.terrain.genExpDownstep(downstep);
            x = obj.terrain.x;
            % this we need to change, zdes should be online adaptable,
            % track where we use it and replace with splines
            zdes = obj.z0 + obj.terrain.zfilter;
            dzdxdes = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz));
            %%%%%%%% downstep
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorFlat(obj) 
            obj.terrain.genFlat;
            %%%%%%%% flat %%%%%%
            x = -1:0.1:10;
            zdes = obj.z0*ones(size(x));
            dzdes = 0*ones(size(x));
            ddzdes = 0*ones(size(x));
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdes, 'ddzdes',ddzdes );       
%              zdes = 0.9338+0.1*sin(t);
%             dzdes = 0.1*cos(t);
%             ddzdes = -0.1*sin(t);
%             obj.desiredbehavior = struct('t', t, 'zdes', zdes, 'dzdes', dzdes, 'ddzdes',ddzdes );
        end
    end
    
    %% ASSIST METHODS
    methods
        %%%%%%%%%%%%% F
        function [Fdes, dFdes] = getDesiredF(obj, t)
            if obj.isDownstep
                if obj.expectedDownstep
                    [Fdes, dFdes] = getDesiredFExpected(obj,t);
                else
                    [Fdes, dFdes] = getDesiredFUnexpected(obj,t);
%                     [Fdes, dFdes] = getDesiredFExpected(obj,t);
%                     [Fdes, dFdes] = getDesiredFUnexpectedAsExpected(obj,t);
                end
            else
                [Fdes, dFdes] = getDesiredFNominal(obj,t);
            end
            % save Fdes for derivative for next iteration
            updatePre(obj,t,Fdes);
        end
        function obj = updatePre(obj,t,Fdes)
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
                t_norm = (tTmp - 0.0) / ...
                    (obj.nominalBeziers.timeMax_SSP - 0.0);
                obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                % eval beziers
                Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,t_norm);
                dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm);
            else
                % DSP: Normalize time for bezier, DSP from 0 to 1
                % 0.05 added to allow GRF to go below zero (by letting
                % the phase go slightly larger than 1)
                tTmp = clamp(obj.stepTime,0.0,obj.TD+obj.TS);
                t_norm = (tTmp - obj.TS) / (obj.TD) + 0.05;
                obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                % eval beziers
                Fdes = zeros(2,1);
                dFdes = zeros(2,1);
                Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,t_norm);
                Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,t_norm);
                dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
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
                        t_norm_VLO = (tTmp - 0.0) / (totalDownstepTimeSSP - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        tTmp = clamp(obj.stepTime,0.0,nominalStepTime);
                        t_norm = (tTmp - 0.0) / (nominalStepTime - 0.0);

                        n = length(obj.downstepBeziersF.exp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase1.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > obj.TS
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        n = length(obj.downstepBeziersF.exp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > (obj.TS+obj.TD)
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/(obj.TS+obj.TD);
                        end
                        t_norm_VLO = (t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        n = length(obj.downstepBeziersF.exp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.exp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
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
                        t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

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
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm,1)/(obj.TD);
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
                        t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

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
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm,1)/(obj.TD);
                    case 3
                        % THERE SHOULD NOT OCCUR A DSP PHASE 3
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        tTmp = clamp(obj.stepTime,0.0,obj.TS+obj.TD);
                        t_norm = (tTmp - obj.TS) / (obj.TD);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        % eval beziers
                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,t_norm);
                        Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,t_norm);
                        dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                        dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
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
                        t_norm = (tTmp - 0.0) / (obj.TS - 0.0);
                        t_norm_VLO = (t_norm - 0.50)/(1.00 - 0.50);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_SSP{i},h);
                        end
%                         Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,1);
                        dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm);
                        obj.downstepHeightDetected = obj.zsw2f;
                    case 2
                        phase = 'phase2';
                        h = obj.downstepHeightDetected;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.TS
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        n = length(obj.downstepBeziersF.unexp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm);
                        dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm);
                    case 3
                        phase = 'phase3';
                        h = obj.downstepHeightDetected;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.nominalBeziers.timeMax
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        t_norm_VLO = (t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm);
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
                            % should be Bezier with $h$ 
                            DSP1min = obj.stepTime;
                            DSP1max = DSP1min + TDdownstep;
                            passedDSP1 = true;
                        end
                        tTmp = clamp(obj.stepTime,DSP1min,DSP1max);
                        t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

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
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                        dFdes(2) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
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
                        t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        n = length(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw);
                        bv_grf_DSP_st = zeros(1,n);
                        bv_grf_DSP_sw = zeros(1,n);
                        for i = 1:n
                            bv_grf_DSP_st(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_st{i},h);
                            bv_grf_DSP_sw(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_DSP_sw{i},h);
                        end
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                        dFdes(2) = obj.mFrac*bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
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
                        t_norm_VLO = (tTmp - 0.0) / (totalDownstepTimeSSP - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        tTmp = clamp(obj.stepTime,0.0,nominalStepTime);
                        t_norm = (tTmp - 0.0) / (nominalStepTime - 0.0);

                        n = length(obj.downstepBeziersF.unexp.phase1.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase1.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.TS
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.TS;
                        end
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        n = length(obj.downstepBeziersF.unexp.phase2.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase2.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        % as we know the splines from VLO to end of
                        % DSP, the phase considered here slightly is
                        % shifted (linearly) by 42%
                        if obj.stepTime > obj.nominalBeziers.timeMax
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        t_norm_VLO = (t_norm - 0.0)/(0.42 - 0.0);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm_VLO];

                        n = length(obj.downstepBeziersF.unexp.phase3.grf_SSP);
                        bv_grf_SSP = zeros(1,n);
                        for i = 1:n
                            bv_grf_SSP(i) = polyval(obj.downstepBeziersF.unexp.phase3.grf_SSP{i},h);
                        end
                        Fdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm_VLO);
                        dFdes = obj.mFrac*bezier2(bv_grf_SSP,t_norm,1)/(obj.TD);
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
                        t_norm = (tTmp - DSP1min) / (DSP1max - DSP1min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

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
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm,1)/(obj.TD);
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
                        t_norm = (tTmp - DSP2min) / (DSP2max - DSP2min) + 0.05;
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

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
                        Fdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm);
                        Fdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm);
                        dFdes(1) = obj.mFrac*bezier2(bv_grf_DSP_sw,t_norm,1)/(obj.TD);
                        dFdes(2) = obj.mFrac*bezier2(bv_grf_DSP_st,t_norm,1)/(obj.TD);
                        return;
                    case 3
                        % THERE SHOULD NOT OCCUR A DSP PHASE 3
                        % DSP: Normalize time for bezier, DSP from 0 to 1
                        tTmp = clamp(obj.stepTime,0.0,obj.TS+obj.TD);
                        t_norm = (tTmp - obj.TS) / (obj.TD);
                        obj.timeNormFLog = [obj.timeNormFLog; t_norm];

                        % eval beziers
                        Fdes = zeros(2,1);
                        dFdes = zeros(2,1);
                        Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,t_norm);
                        Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,t_norm);
                        dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                        dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
                        return;
                end
            end
        end
        
        
        %%%%%%%%%%%%% z
        function [zdes, dzdes, ddzdes] = getDesiredZ(obj, t)
            if ~obj.isDownstep
                [zdes,dzdes,ddzdes] = getDesiredZNominal(obj,t);
            else
                % downstep getting
                if obj.expectedDownstep 
                    [zdes,dzdes,ddzdes] = getDesiredZExpected(obj,t);
                else
%                     [zdes,dzdes,ddzdes] = getDesiredZUnexpected(obj,t);
                    [zdes,dzdes,ddzdes] = getDesiredZUnexpectedAsExpected(obj,t);
                end
            end
        end
        
        function [zdes, dzdes, ddzdes] = getDesiredZNominal(obj,t)            
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                if obj.stepTime > (obj.TD+obj.TS)
                    t_norm = 1;
                else
                    t_norm = obj.stepTime/(obj.TD+obj.TS);
                end
                obj.timeNormZLog = [obj.timeNormZLog; t_norm];

                if obj.stepTimeVLO > (obj.TD+obj.TS)
                    t_norm_VLO = 1;
                else
                    t_norm_VLO = obj.stepTimeVLO/(obj.TD+obj.TS);
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
                    zdes = factors(index)*(bezier2(obj.nominalBeziers.bv_zcom,t_norm)-zdesMean) + zdesMean;
                    dzdes = factors(index)*bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
                    ddzdes = factors(index)*bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                else
                    factor = 0.5;
                    zdes = factor*(bezier2(obj.nominalBeziers.bv_zcom,t_norm)-zdesMean) + zdesMean;
                    dzdes = factor*bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
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
                            t_norm_VLO = 1;
                        else 
                            t_norm_VLO = obj.stepTimeVLO/totalDownstepTime;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal;
%                         dzdes = bezier2(bv_dzcom,t_norm_VLO);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(totalDownstepTime);
                        return;
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;

                        if obj.stepTime > obj.nominalBeziers.timeMax
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm];

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
                        zdes = bezier2(bv_zcom,t_norm) - obj.deltaNominal;
%                         dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,t_norm,1)/(obj.TS+obj.TD);
%                         ddzdes = bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                        return; 
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > (obj.TS/2)
                            t_norm_VLO = 1;
                        else
                            t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal;
%                         dzdes = bezier2(bv_dzcom,t_norm_VLO);
%                         ddzdes = bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(totalDownstepTime);
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
                        TDdownstep = obj.TD/2;
                        if passedSSP1
                            timeMax = impactTime + TDdownstep;
                        else
                            timeMax = obj.stepTimeVLO + TDdownstep;
                        end

                        if obj.stepTimeVLO > timeMax
                            t_norm_VLO = 1;
                        else 
                            t_norm_VLO = obj.stepTimeVLO/timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal + (1-t_norm_Downstep)*obj.deltaDownstep;
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(timeMax) + ...
                                (bezier2(bv_zcom,t_norm_VLO)-bezier2(bv_zcom2,t_norm_VLO))/0.005*obj.dzsw2f;
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(timeMax) + ...
                                 (bezier2(bv_dzcom,t_norm_VLO)-bezier2(bv_dzcom2,t_norm_VLO))/0.005*obj.dzsw2f;
                        % gradient velocity is the velocity that occurs due
                        % to traversing the surface
%                         gradientVelocity = (0.7981 - 0.8209)/(4.511 - 4.476);
%                         dzdes = gradientVelocity/2;
%                         dzdes = -0.1798;
%                         dzdes = bezier2(bv_dzcom,t_norm_VLO);
%                         ddzdes = 0.0;
                        
                        if obj.stepTime > (obj.TD+obj.TS)
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/(obj.TD+obj.TS);
                        end
                        [zcomNominal,~,~] = obj.getDesiredZNominal(t);
                        obj.zcomNominalLog = [obj.zcomNominalLog;
                                              obj.xcom zcomNominal 0.0 t_norm];
                        obj.zcomDownstepLog = [obj.zcomDownstepLog;
                                               obj.xcom zdes h t_norm_VLO];
                        return;                            
                    case 2
                        phase = 'phase2';
                        h = obj.downstepHeightDetected;

                        if obj.stepTime > obj.nominalBeziers.timeMax
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm];

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
                        zdes = bezier2(bv_zcom,t_norm) - obj.deltaNominal;
%                             dzdes = bezier2(bv_dzcom,t_norm);
                        dzdes = bezier2(bv_zcom,t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,t_norm,1)/(obj.TS+obj.TD);
                        return;
                    case 3
                        phase = 'phase3';
                        h = obj.downstepHeightDetected;

                        if obj.stepTime > (obj.TS/2)
                            t_norm_VLO = 1;
                        else
                            t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal;
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(obj.TS/2);
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(obj.TS/2);                           
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
                        h = obj.knownDownstepHeight; % obj.zsw2f
                        [~,idx] = min(abs(obj.swingZbehavior.z - h));
                        nominalStepTime = obj.swingZbehavior.t(idx);
                        if isempty(totalDownstepTime)
                            totalDownstepTime = obj.TD + (nominalStepTime-obj.TS/2);
                        end

                        if obj.stepTimeVLO > totalDownstepTime
                            t_norm_VLO = 1;
                        else 
                            t_norm_VLO = obj.stepTimeVLO/totalDownstepTime;
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal + (1-t_norm_Downstep)*obj.deltaDownstep;
%                         dzdes = bezier2(bv_dzcom,t_norm_VLO);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(totalDownstepTime);
                        return;
                    case 2
                        phase = 'phase2';
                        h = obj.knownDownstepHeight;

                        if obj.stepTime > obj.TS+obj.TD
                            t_norm = 1;
                        else
                            t_norm = obj.stepTime/(obj.TS+obj.TD);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm];

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
                        zdes = bezier2(bv_zcom,t_norm) - obj.deltaNominal;
%                         dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
%                         ddzdes = 0.0;
                        dzdes = bezier2(bv_zcom,t_norm,1)/(obj.TS+obj.TD);
                        ddzdes = bezier2(bv_dzcom,t_norm,1)/(obj.TS+obj.TD);
%                         ddzdes = bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                        return; 
                    case 3
                        phase = 'phase3';
                        h = obj.knownDownstepHeight;
                        
                        if obj.stepTime > (obj.TS/2)
                            t_norm_VLO = 1;
                        else
                            t_norm_VLO = obj.stepTime/(obj.TS/2);
                        end
                        obj.timeNormZLog = [obj.timeNormZLog; t_norm_VLO];

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
                        zdes = bezier2(bv_zcom,t_norm_VLO) - obj.deltaNominal + (t_norm_VLO)*(0.8795-0.8695);
%                         dzdes = bezier2(bv_dzcom,t_norm_VLO);
%                         ddzdes = bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                        dzdes = bezier2(bv_zcom,t_norm_VLO,1)/(totalDownstepTime);
                        ddzdes = bezier2(bv_dzcom,t_norm_VLO,1)/(totalDownstepTime);
                        return;                            
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
        end
        
        
        %%%%%%%%%%%%%% swing
        function [qns, dqns] = stepLengthUpdateSwingLegAngle(obj, stepLength, dstepLength, varargin)
            % based on the updated step length, update the swing leg angle
            % assume it is perfectly tracked due to the trivial dynamics on
            % the swing leg.
            
            %  syms qns dqns
            %   eqn1 = stepLength == obj.polar.rs*sin(obj.polar.qs) + obj.polar.rns*(-sin(qns));
            %  eqn2 = dstepLength ==  obj.polar.drs*sin(obj.polar.qs) + obj.polar.rs*cos(obj.polar.qs)*obj.polar.dqs +...
            %                          obj.polar.drns*(-sin(qns)) + obj.polar.rns*(-cos(qns))*dqns;
            
            % [qns, dqns] = solve([eqn1, eqn2],[qns dqns]);
            if isempty(varargin)
                qns = asin((obj.polar.rs*sin(obj.polar.qs) - stepLength)/  obj.polar.rns);
                dqns  = -( obj.polar.drs*sin(obj.polar.qs) + obj.polar.rs*cos(obj.polar.qs)*obj.polar.dqs +...
                    obj.polar.drns*(-sin(qns)) - dstepLength)/ (obj.polar.rns*(-cos(qns)));
                % solve for
                obj.polar.qns = qns;
                obj.polar.dqns = dqns;
            else % vector operation
                % rs qs, rns,
                % drs dqs, drns
                rs = varargin{1};
                qs = varargin{2};
                rns = varargin{3};
                drs = varargin{4};
                dqs = varargin{5};
                drns = varargin{6};
                qns = asin((rs.*sin(qs) - stepLength)./rns);
                dqns  = -( drs.*sin(qs) + rs.*cos(qs).*dqs + drns.*(-sin(qns)) -dstepLength)./ (rns.*(-cos(qns)));
            end
        end

        function SwingFootZconstruct(obj)
            Tf = obj.TS;
            period = 0.001;
            Tvec = 0:period:Tf;
            HeightZ = obj.swingHeight;
            %%%% swingZ, velocity is a sinusoidal
            swingZvel = HeightZ*pi/Tf*sin(Tvec/Tf*2*pi);
            makeSureIntrusion = 0.001;
            swingZ = HeightZ/2*(-cos(Tvec/Tf*2*pi)+1) - makeSureIntrusion*Tvec/Tf;
            
             Tvec = 0:period:2*Tf;
            swingZpos = HeightZ*cos(Tvec/Tf*pi - pi/2);
            swingZvel = -HeightZ*pi/Tf*sin(Tvec/Tf*pi - pi/2);
          
            obj.swingZbehavior = struct('t',Tvec,'z',swingZpos,'dz',swingZvel);
            
            %% with bezier expected downstep
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
            plot(Tvec,swingZpos)
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
            
            swingZbehavior.NS = obj.swingZbehavior;
            swingZbehavior.OS = obj.swingZbehaviorOS;
            swingZbehavior.US = obj.swingZbehaviorUS;
            save('data/outputs/swingZbehavior.mat','swingZbehavior')
        end
        
        %%%%%%%%%%%%%% xcom phasing stuff
        function [zdes, dzdes, ddzdes] = getDesiredZXcomBased(obj,t)
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                if obj.isDownstep
                    % ain't gonna happen
                else
                    [zdes, dzdes, ddzdes] = combinedBezierZ(obj,t);                    
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
        end
        function [zcom,dzcom,ddzcom] = combinedBezierZ(obj,t)
            load('data/outputs/bezierStepTimes.mat')
            Ts1 = polyval(bezierStepTimes.exp.Ts1,0.0);
            Ts2 = polyval(bezierStepTimes.exp.Ts2,0.0);
            Ts3 = polyval(bezierStepTimes.exp.Ts3,0.0);
            Tt = Ts1 + Ts2 + Ts3;
            xcomDesired = obj.stepLdes;
            
            if ~obj.isDownstep
                Txcom = obj.xcom2f * xcomDesired/(Tt/2);
            else
                Txcom = obj.xcom2f * xcomDesired/(Tt/2);
            end
            
            if Txcom > 0 && Txcom <= Ts1
                % use first bezier
                phase = 'phase1';
                xcom_norm = (Txcom - 0.0)/(Ts1 - 0.0);
            elseif Txcom > Ts2 && Txcom <= Ts3
                phase = 'phase2';
                xcom_norm = (Txcom - Ts2)/(Ts3 - Ts2);
            else
                phase = 'phase3';
                Txcom = clamp(Txcom,Ts2,Ts3);
                xcom_norm = (Txcom - Ts2)/(Ts3 - Ts2);
            end
            
            n = length(obj.downstepBeziersZ.exp.(phase).zcom);
            bv_zcom = zeros(1,n);
            for i = 1:n
                bv_zcom(i) = polyval(obj.downstepBeziersZ.exp.(phase).zcom{i},0.0);
            end
            n = length(obj.downstepBeziersZ.exp.phase1.dzcom);
            bv_dzcom = zeros(1,n);
            for i = 1:n
                bv_dzcom(i) = polyval(obj.downstepBeziersZ.exp.(phase).dzcom{i},0.0);
            end
            zcom = bezier2(bv_zcom,xcom_norm);
            dzcom = bezier2(bv_dzcom,xcom_norm);
            ddzcom = 0;
        end
        
    end
end