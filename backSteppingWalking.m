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
        
        % Length dependent stiffness and damping
        % now K is called as K(L,obj.Kparam) 
        g = 9.81;
        m = 66.5138;
        Kparam = [769.9856 911.3193 1.3764e4];
        Dparam = 100;
        KparamSim = [769.9856 911.3193 1.3764e4];
        DparamSim = 100;
        
        K = @(L,Kparam) polyval(Kparam,L);
        D = @(L,Dparam) polyval(Dparam,L);
        
        nominalBeziers;
        downstepBeziersF;
        downstepBeziersZ;
        
        useIncreasingVelocity = false;
        useIncreasingDeviation = false;
        useDecreasingRelaxation = false;
        
        useSwingFootDetection = false;
        xsw = 0.0;
        zsw = 0.0;
        xcom = 0.0;
        detectionHeight = -0.005;
        
        stepCnt = 1;
        stepTime = 0.0;
        stepsToTrueDesired = 8;
        useHumanZ = false;
        useHumanF = false;
        
        isDownstep = false;
        downstepStep = 0;
        downstepTime = 0.0;
        downstepDuration = 0.0;
        downstepHeightOffset = 0.0;
        VLOtime = 0.0;
        bv_SSP2 = struct();     % just for Fdes and dFdes of SSP2
        bv_SSP3 = struct();     % just for zcom and dzcom of SSP3
        zdesPrev = struct;
        FdesPrev = struct;
        
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
        
        isDownstepLog = [];
        downstepStepLog = [];
        
        
        stepTimelog = [];
        
        
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
        desiredGRF
        terrain
        z0
        
        terrainType %% Flat Slope Rough
   end
    
    properties  %% control
         %%% linear controller gain
        Kp = 10; %10 %%%% PD gains for leg length control
        Kd = 7; %10
        epsilon = .1; % 0.1;
        c_relax_DSP = 0.15; %0.3; %%% DSP, stance force relax coef
        c_relax_SSP = 0.15;
        deltaF = 50; % 10, 20 
        Kbackstepping = 100; 
        gama = 10;  %%% clf : dV< -gama*V; 
        beta = 500  %%% CBF: dh > - beta*(h- Fmin) with h = Fs in SSP; %% larger 
        Fmin = 20; 
        ddLmax = 500
        
        Fext = 0 %%% external push force
        LIP
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
        function obj = backSteppingWalking(system)
            if isequal(system,'human')
                tmpNBH = load('data/outputs/nominalBeziersHuman.mat');
                tmpDBHF = load('data/outputs/bezierInterpolation.mat');
                tmpDBHZ = load('data/outputs/bezierZcomInterpolation.mat');
                obj.nominalBeziers = tmpNBH.nominalBeziers;
                obj.downstepBeziersF  = tmpDBHF.downstepBeziers;
                obj.downstepBeziersZ  = tmpDBHZ.downstepBeziers;
                
                obj.m = 66.5138;
                obj.Kparam = [769.9856 911.3193 1.3764e4];
                obj.Dparam = 100;
                obj.KparamSim = obj.Kparam;
                obj.DparamSim = obj.Dparam;
            else % 'cassie'
                disp("CASSIE NOT READY YET!")
                throw("..")
%                 tmpNBC = load('data/outputs/nominalBeziersCassie.mat');
%                 tmpDBC = load('data/outputs/downstepBeziersCassie.mat');
%                 obj.nominalBeziers  = tmpNBC.nominalBeziers;
%                 obj.downstepBeziers = tmpDBC.downstepBeziers;
%                 
%                 obj.m = 31;
%                 obj.Kparam = [23309 0 -55230 48657 -9451];
%                 obj.Dparam = [348 0 -824 726 -141];
%                 obj.KparamSim = obj.Kparam;
%                 obj.DparamSim = obj.Dparam;
            end
            
            SSPfrac = (obj.nominalBeziers.timeMax_SSP)/obj.nominalBeziers.timeMax;
            obj.TS = SSPfrac*obj.nominalBeziers.timeMax;
            obj.TD = (1-SSPfrac)*obj.nominalBeziers.timeMax;
            obj.z0 = mean(bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1));
            obj.TF = 6;     % total simulation time

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
            dxcom = mean(bezier2(obj.nominalBeziers.bv_dxcom,0:0.01:1));
            if obj.useIncreasingVelocity
                dxN = 0.5;      % starting velocity
                desiredVelocity = [dxN dxN dxN linspace(dxN,dxcom,obj.stepsToTrueDesired)];
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
                
                if T(end) < t+period && obj.NcontactLegs == 1
                    % reset time if from DSP to SSP
                    obj.stepTime = 0.0;
                    % increase the step count
                    obj.stepCnt = obj.stepCnt + 1;
                    % reset the desired velocity based on tnow
                    if obj.useIncreasingVelocity
                        obj.setVelocityFromBezier();
                    end
                    % if we're at downstep, increase stepCnt for downstep
                    if obj.isDownstep
                        obj.downstepStep = obj.downstepStep + 1;
                    end
                end
                
                if testFootNegative
                    % switch to true if this is the first time
                    if ~obj.isDownstep
                        obj.isDownstep = true;
                        obj.downstepStep = 1;
                        obj.downstepTime = ts;
                    end
                end
                
                if ~testFootNegative && obj.downstepStep == 3 && obj.NcontactLegs == 2
                    % go back to normal if we've had three steps and the
                    % swing foot is again positive
                    obj.isDownstep = false;
                    obj.downstepStep = 0;
                end
                
                Xaug = obj.stateAugmentation(ts, X0);

                obj.Xsol = [obj.Xsol; Xaug'];
                obj.Tsol = [obj.Tsol, T(end)];
                obj.usol = [obj.usol, u];
                obj.GRFsol = [obj.GRFsol, [obj.polar.Fs; obj.polar.Fns]];
                
                obj.zdesBezier   = [obj.zdesBezier; zdesStruct.zdes];
                obj.dzdesBezier  = [obj.dzdesBezier; zdesStruct.dzdes];
                obj.ddzdesBezier = [obj.ddzdesBezier; zdesStruct.ddzdes];
                obj.isDownstepLog = [obj.isDownstepLog; obj.isDownstep];
                obj.downstepStepLog = [obj.downstepStepLog; obj.downstepStep];
                
                obj.stepTimelog  = [obj.stepTimelog; obj.stepTime];
                
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
            else %%% next domain is DSP
                %%%% preimpact event 
                %obj.LIP.MPC_controller('MPCtrackingVelocityP1', ts, [obj.polar.x2f;obj.polar.dx])%%% MPC on LIP 
                obj.LIP.updateLIPstates(ts); 
                obj.LIP.calculateNominalTrajPerStep(ts, obj.TS, obj.TD, obj.polar, obj.curstepLength) 
                obj.logDiscreteState(ts); 
                X0 = obj.virtualImpactEvent(X0, obj.curstepLength);
                obj.FliftOff = obj.polar.Fs; 
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
                    terrainZ = interp1(obj.terrain.x, obj.terrain.z, swingx);
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
            obj.xsw = obj.polar.x - obj.polar.Lns*sin(obj.polar.qns); 
            obj.zsw = obj.polar.z - obj.polar.Lns*cos(obj.polar.qns);
            obj.xcom = obj.polar.x;
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
        end
        
        function ifpen = ifSwingFootPenetrating(obj, t)
            ifpen = 0; 
            if obj.NcontactLegs== 1
                if t - obj.TimeStamp(end) > obj.TS/2
                    swingx = obj.polar.x + obj.polar.rns*(-sin(obj.polar.qns));
                    swingZ = obj.polar.z - obj.polar.rns*cos(obj.polar.qns);
                    terrainZ = interp1(obj.terrain.x, obj.terrain.z, swingx);
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
                if t - obj.TimeStamp(end) > obj.TS/2
                    swingZ = obj.polar.z - obj.polar.rns*cos(obj.polar.qns);
                    if swingZ < -0.01;
                        ifneg = 1;
                    end
                end
            end
        end
    end
    
    methods %%% analysis
        function plot(obj)
            X = obj.Xsol;
            t = obj.Tsol;
            
            x = X(:,1);    dx = X(:,2);
            z = X(:,3);    dz = X(:,4);
            L1 = X(:,5);   dL1 = X(:,6); %original left leg
            L2 = X(:,7);   dL2 = X(:,8);
            sL1 = X(:,9);  dsL1 = X(:,10);
            sL2 = X(:,11); dsL2 = X(:,12);
            r1 = X(:, 13); dr1 = X(:, 14);
            r2 = X(:, 15);  dr2 = X(:, 16);
            q1 = X(:, 17);  dq1 = X(:, 18);
            q2 = X(:, 19);  dq2 = X(:, 20);
            
            %%%%%%%%%%% task %%%%%%%%%%%
      
            figure; hold on; grid on;
            plot(t,obj.stepTimelog,'r')
            xlabel('t'); ylabel('stepTime')
            title('stepTime during simulation')
            
            
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
            subplot(3,1,1); hold on; grid on;
            plot(t,z,'r')
            plot(obj.desiredbehavior.x, obj.desiredbehavior.zdes, 'b')
            plot(t,obj.zdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(t), max(t)])
            title('t-z');
            
            subplot(3,1,2); hold on; grid on;
            plot(t,dz,'r')
            plot(x,dx.*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x), 'b')
            plot(t,obj.dzdesBezier,'g')
            legend('act', 'des', 'desBezier')
            xlim([min(t), max(t)])
            title('t-dz')
            
            subplot(3,1,3); hold on; grid on;
            plot(t, obj.GRFsol(1,:), 'r')
            plot(t, obj.GRFsol(2,:), 'b') 
            plot(obj.GRFsbound(:,1),obj.GRFsbound(:,2),'g')
            plot(obj.GRFsbound(:,1),obj.GRFsbound(:,3),'g')
            plot(obj.GRFnsbound(:,1),obj.GRFnsbound(:,2),'g')
            plot(obj.GRFnsbound(:,1),obj.GRFnsbound(:,3),'g')
            xlim([min(t), max(t)])
            title('F (N)')
            legend('Fs', 'Fns', 'bound');
            
            

            %%%%%%%%%%%% leg behavior %%%%%%%%%%%%%
            figure
            subplot(3,2,1); hold on; grid on;
            plot(t, sL1, 'r')
            plot(t, sL2, 'b')
            legend('sL1', 'sL2')
            title('s');
             
            subplot(3,2,2); hold on; grid on;
            plot(t, L1, 'r')
            plot(t, L2, 'b')
            title('L')
            legend('L1', 'L2')

            subplot(3,2,3); hold on; grid on;
            plot(t, dL1, 'r')
            plot(t, dL2, 'b')
            title('dL')
            legend('dL1', 'dL2')
            
            subplot(3,2,4); hold on; grid on;
            plot(t, dsL1, 'r')
            plot(t, dsL2, 'b')
            title('ds');
            legend('dsL1', 'dsL2');

            subplot(3,2,5); hold on; grid on;
            plot(t, obj.usol(1,:), 'r')
            plot(t, obj.usol(2,:), 'b')
            title('ddL')
            legend('ddL1', 'ddL2');

            subplot(3,2,6); hold on; grid on;
            plot(t, obj.GRFsol(1,:), 'r')
            plot(t, obj.GRFsol(2,:), 'b') 
            title('F (N)')
            legend('Fs', 'Fns');
            unDeformedFootPosZ1 = z - L1.*cos(q1);
            unDeformedFootVelZ1 = dz - dL1.*cos(q1) + L1.*sin(q1).*dq1;
            
            unDeformedFootPosZ2 = z - L2.*cos(q2);
            unDeformedFootVelZ2 = dz - dL2.*cos(q2) + L2.*sin(q2).*dq2;
            
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
            try
                figure,
                N = length(obj.LIP.HLIP.t);
                subplot(2,2,1)
                plot(obj.LIP.HLIP.t, obj.LIP.HLIP.x2f, 'r', obj.LIP.HLIP.t, obj.LIP.HLIP.pred_p, 'b'); 
                title('position')
                subplot(2,2,2)
                plot(obj.LIP.HLIP.t, obj.LIP.HLIP.dx, 'r', ...
                    obj.LIP.HLIP.t, obj.LIP.HLIP.pred_v, 'b');            
                title('velocity')
                subplot(2,2,3)
                plot(obj.LIP.HLIP.x2f, obj.LIP.HLIP.dx) ; title('phase')
                            subplot(2,2,4)
                plot(obj.LIP.HLIP.t, obj.LIP.HLIP.u, 'r'); title('step size') 

                %%% discrete
                figure,
                subplot(3,1,1),
                title('p')
                plot(obj.LIP.robotXF.t, obj.LIP.robotXF.p, 'ro-', obj.LIP.robotXF.t, obj.LIP.XLIPlist(1,:), 'bo-');
                subplot(3,1,2),
                title('v')
                plot(obj.LIP.robotXF.t, obj.LIP.robotXF.v, 'ro-', obj.LIP.robotXF.t, obj.LIP.XLIPlist(2,:), 'bo-');
                subplot(3,1,3),           
                title('u')
                plot(obj.LIP.robotXF.t, obj.LIP.robotXF.u, 'ro-', obj.LIP.robotXF.t, obj.LIP.targetStepLengthVec, 'bo-');
                figure,        
                plot(obj.LIP.HLIP.t, obj.LIP.HLIP.u, 'r', obj.LIP.HLIP.t, obj.LIP.SLS_ulog, 'b'); title('step size') 
            catch
                disp("not plotting something")
            end
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
            Tsol = obj.Tsol;
            X = obj.Xsol;
            animData = struct;
            [~, index] = unique(Tsol);
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
            
            animData.t = {Tsol(index)};
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
            %%% F = f2 + g2*u;
            f1 = [obj.polar.dz - dzdes; -obj.g - ddzdes];
            g1 = [0; 1/obj.m*cos(qs)];
            g2 = obj.D(Ls,obj.Dparam); f2 = obj.K(Ls,obj.Kparam)*ds - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + r*dqs^2 + 1/obj.m*Fs);
            
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
            rs = obj.polar.rs;
            drs = obj.polar.drs;
            s = obj.polar.sLs;
            ds = obj.polar.dsLs;
            qs = obj.polar.qs;
            dqs = obj.polar.dqs;
            Ls = obj.polar.Ls;
            sLs = obj.polar.sLs;
            dsLs = obj.polar.dsLs;
            
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
            
            g2 = obj.D(Ls,obj.Dparam); 
            f2 = obj.K(Ls,obj.Kparam)*ds - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + rs*dqs^2 + 1/obj.m*Fs);
        
           
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

          %  obj.gama = min([eigQ(1,1), eigQ(2,2)])/max([eigP(1,1), eigP(2,2)]); 
            Fbar_fbIO = obj.m/cos(qs)*(obj.g + ddzdes + KIO*eta);
            dFbar_fbIO = obj.m/cos(qs)*(dddzdes + KIO*deta) + obj.m*(obj.g + ddzdes + KIO*eta)*sec(qs)*tan(qs)*dqs; 
            
            udes =( -partial_V_eta*g1 - obj.Kbackstepping*(Fs - Fbar_fbIO))/pz + [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd]*(f1 + g1*Fs);
            udes = (udes - f2)/g2; 
            %%%%%%%%%%%%%%% CLF constraint construction %%%%%%%%%%%%%%
            z = Fs - Fbar_fbIO;
            V = V_eta + 1/2*pz*z^2; 
            dVeta = partial_V_eta*deta; 
            %%%% Vdot := LvF + LvG*u = dVeta + pz*z*dz 
            %%%% dz = dFs - dFbar_fbIO = f2 + g2*u - dFbar_fbIO; 
            %%%% dV < -gama*V + delta; LvF + LvG*u <  -gama*V + delta;  Aclf*u < bclf
            LvF = dVeta + pz*z*(f2 - dFbar_fbIO); 
            LvG = pz*z*g2; 
            Aclf =  [LvG, -1e+3];
            Bclf = -obj.gama*V - LvF; 
            
            %%%%%%%%%%%%%%%%%%%% CBF constraint construction %%%%%%%%%%%%
            ddrs = Fs/obj.m - obj.g*cos(qs) + rs*dqs^2;
            gs = obj.D(Ls,obj.Dparam); 
            fs = obj.K(Ls,obj.Kparam)*dsLs - obj.D(Ls,obj.Dparam)*ddrs;
            
            deltaF = obj.deltaF;
            if obj.useHumanF
                if obj.useDecreasingRelaxation
                    c_relaxs = [0.8 0.8 0.8 linspace(0.8,obj.c_relax_SSP,obj.stepsToTrueDesired)];
                    index = length(c_relaxs);
                    if obj.stepCnt < length(c_relaxs)
                        index = obj.stepCnt;
                    end
                    c = c_relaxs(index);
                    if obj.isDownstep
                        c = c + 0.15;
                    end
                else
                    c = obj.c_relax_SSP;
                end
                
                hs = (c*Fsdes + deltaF)^2 - (Fs - Fsdes)^2;
                Acbf = [2*(Fs-Fsdes)*gs, 0];
                Bcbf = c^2*2*Fsdes*dFsdes +  2*c*deltaF*dFsdes- 2*(Fs - Fsdes)*(fs - dFsdes) + obj.beta*hs;
                
                %%%% log the admissible force set: 
                obj.GRFsbound = [obj.GRFsbound; [t, (1-c)*Fsdes - deltaF, (1+c)*Fsdes + deltaF]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, (1-c)*Fsdes - deltaF, (1+c)*Fsdes + deltaF]]; 
            else
                Fsdes = 0;
                hs = Fs - obj.Fmin; 
                % dh = f2 + g2*u > - beta*h, 
                % Acbf*u < bcbf
                Acbf = [-g2, 0];
                Bcbf = f2 + obj.beta*hs;
                
                %%%% log the admissible force set: 
                obj.GRFsbound = [obj.GRFsbound; [t, 0, 0]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0]]; 
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
           %  p = 1; 
            H = [1,0;0,p]; 
%             H = [1e+4 + LvG^2,0;0,p]; 
%             G = [LvF;0];
%             G = [-udes;0];
            G = [0;0];
            nWSR = 10000;
            
            options = optimoptions('quadprog','Display','off');
            [usol,fval,exitflag, numiter] = quadprog(H,G, A, ubA, [], [], lb, ub,[],options);
%             if exitflag ~= 1
%                 disp("--- QP ERROR!!! ---")
%                 usol = zeros(2,1);
%             end
            
            delta = usol(2); 
            u_nonstance = obj.swingLegControl(t); 
            u = [usol(1); u_nonstance];
            
            dV = LvF + LvG*usol(1); 
            cbfH = Acbf*usol - Bcbf;
            cbfH = [cbfH; 0;]; 
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = OutputPD_QP(obj, t)
            r = obj.polar.rs;
            dr = obj.polar.drs;
            s = obj.polar.sLs;
            ds = obj.polar.dsLs;
            qs = obj.polar.qs;
            dqs = obj.polar.dqs;
            Ls = obj.polar.Ls;
            
            % Compute desired zcom, either constant (from filtered ground
            % profile) or from bezier splines
            [zdes, dzdes, ddzdes] = obj.getDesiredZ(t);
            zdesStruct.zdes = zdes;
            zdesStruct.dzdes = dzdes;
            zdesStruct.ddzdes = ddzdes;
            
            % Compute desired GRF from splines
            FdesStruct = struct;
            
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
            f1 = [obj.polar.dz - dzdes; -obj.g - ddzdes];
            g1 = [0; 1/obj.m*cos(qs)];
            deta = f1 + g1*Fs; 
            
            g2 = obj.D(Ls,obj.Dparam); f2 = obj.K(Ls,obj.Kparam)*ds - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + r*dqs^2 + 1/obj.m*Fs);
            
            KIO =  [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd];
          
            Fbar_fbIO = obj.m/cos(qs)*(obj.g + ddzdes + KIO*eta);
             %%%%%%%%%%%% QP construction %%%%%%%%%%%%%%%%%%%%%%%%
             %%%% ||g1*F + f1 - K*eta||^2 = ||F - Fbar_fbIO||^2
             H = 1;
             G = -Fbar_fbIO;
             A = -1; ubA = 0;
             nWSR = 10000;
             options = optimoptions('quadprog','Display','off');
             [Fsol,fval,exitflag, numiter] = quadprog(H, G, A, ubA, [], [], [], [], [], options);
             
            Kf = -4; 
            udes = Kf*(Fs - Fsol);
            us = (udes - f2)/g2; 
      
            %%%%%%%%%%%%%%%%%%%% CBF constraint construction %%%%%%%%%%%%
            %%%%%%%%%%% dh = f2 + g2*u > - beta*h, Acbf*u < bcbf
            u_nonstance = obj.swingLegControl(t);
            u = [us; u_nonstance];
            V = 0;
            dV = 0;
            delta = 0;
            cbfH = [0; 0;];
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = DSPbackSteppingCLFCBF(obj, t)
            rs = obj.polar.rs;            rns = obj.polar.rns;
            drs = obj.polar.drs;          drns = obj.polar.drns;
            sLs = obj.polar.sLs;          sLns = obj.polar.sLns;
            dsLs = obj.polar.dsLs;        dsLns = obj.polar.dsLns;
            qs = obj.polar.qs;            qns = obj.polar.qns;
            dqs = obj.polar.dqs;          dqns = obj.polar.dqns;
            Ls = obj.polar.Ls;            Lns = obj.polar.Lns;

            % Compute desired zcom, either constant (from filtered ground
            % profile) or from bezier splines
            [zdes, dzdes, ddzdes] = obj.getDesiredZ(t);
            zdesStruct.zdes = zdes;
            zdesStruct.dzdes = dzdes;
            zdesStruct.ddzdes = ddzdes;
            
            % Compute desired GRF from splines
            FdesStruct = struct;
            
            tInDomain = t - obj.TimeStamp(end); 
            tInDomain(tInDomain<0) = 0; 
            Fsdes = obj.FliftOff*interp1(obj.desiredGRF.t, obj.desiredGRF.Fmax, tInDomain);
            dFsdes = obj.FliftOff*interp1(obj.desiredGRF.t, obj.desiredGRF.dFmax, tInDomain);

            dddzdes = 0; 
            Fs = obj.springForce(Ls, sLs, dsLs);
            Fns = obj.springForce(Lns, sLns, dsLns);
            
            %%%%%%%%%%%%%%%% optionA: Fs decreases,Fns as output
            %%% eta1 = z - zdes; eta2 = dz - dzdes;
            %%% ddz = (obj.polar.Fs*cos(obj.polar.qs) + obj.polar.Fns*cos(obj.polar.qns))/obj.m - obj.g;
            %%% ddeta = ddz - ddzdes =  -g + 1/obj.m*Fs*cos(qs) + 1/obj.m*Fns*cos(qns) - ddzdes = ddeta_FB_IO =  -kp*eta -kd*deta
            %%% ddrs = (Fs + Fns*cos(qs - qns))/obj.m - obj.g*cos(qs) + rs*dqs^2;
            %%% ddrns = (Fns + Fs*cos(qs - qns))/obj.m - obj.g*cos(qns) + rns*dqns^2;
            %%% dFs = K*dsLs + D*ddsLs = K*dsLs + D*(u1 - ddrs) = D*u1 + K*dsLs -
            %%% D*(-g*cos(qs) + rs*dqs^2 + (Fs + Fns*cos(qs - qns))/obj.m) = fs + gs*u1;
            %%% dFns = K*dsLns + D*ddsLns = K*dsLns + D*(u2 - ddrns) = D*u2 + K*dsLns -
            %%% D*(-g*cos(qns) + rns*dqns^2 + (Fns + Fs*cos(qs - qns))/obj.m ) = fns + gns*u2;
            eta = [obj.polar.z - zdes;
                   obj.polar.dz - dzdes];
            %%% deta = f0 + g1*F;
            f0 = [obj.polar.dz - dzdes; 
                  -obj.g - ddzdes];
            g0 = [0,              0; 
                  cos(qs)/obj.m,  cos(qns)/obj.m];
            deta = f0 + g0*[Fs; 
                            Fns]; 
            g02 = [0;  
                   cos(qns)/obj.m];
               
            g1 = obj.D(Ls,obj.Dparam);  
            f1 = obj.K(Ls,obj.Kparam)*dsLs   - obj.D(Ls,obj.Dparam)*(-obj.g*cos(qs) + rs*dqs^2 + (Fs+Fns*cos(qs - qns))/obj.m);
            g2 = obj.D(Lns,obj.Dparam); 
            f2 = obj.K(Lns,obj.Kparam)*dsLns - obj.D(Lns,obj.Dparam)*(-obj.g*cos(qns) + rns*dqns^2 + (Fns+Fs*cos(qs - qns))/obj.m);
            
            k = 1000; 
            k = 100;
            us = 1/g1*(-f1 - k*(Fs - Fsdes) + dFsdes);
            dFs = g1*us + f1;
            %%% control the stance Force %%%%
            V_eta = 1/2*transpose(eta)*eta;
            partial_V_eta = transpose(eta);
            
            % K > min(eig([0,1;kp, kd]))
            pz = 1; %%%%%% coef on the cost function 
            KIO =  [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd];
            
            Fnsbar_fbIO = obj.m/cos(qns)*(obj.g - Fs*cos(qs)/obj.m + ddzdes + KIO*eta);
            dFnsbar_fbIO = obj.m/cos(qns)*(dddzdes + KIO*deta - dFs*cos(qs)/obj.m + Fs*sin(qs)*dqs/obj.m) + ...
                           obj.m*(obj.g + ddzdes - Fs*cos(qs)/obj.m + KIO*eta)*sec(qns)*tan(qns)*dqns; 
            
            u_ns_des =( -partial_V_eta*g02 - obj.Kbackstepping*(Fns - Fnsbar_fbIO))/pz + [-1/(obj.epsilon^2)*obj.Kp, - 1/obj.epsilon*obj.Kd]*deta;
            u_ns_des = (u_ns_des - f2)/g2; 
            %%%%%%%%%%%%%%% CLF constraint construction %%%%%%%%%%%%%%
            z = Fns - Fnsbar_fbIO;
            V = V_eta + 1/2*pz*z^2; 
            dVeta = partial_V_eta*deta; 
            %%%% Vdot := LvF + LvG*u = dVeta + pz*z*dz 
            %%%% dz = dFs - dFbar_fbIO = f2 + g2*u - dFbar_fbIO; 
            %%%% dV < -gama*V + delta; LvF + LvG*u <  -gama*V + delta;  Aclf*u < bclf
            LvF = dVeta + pz*z*(f2 - dFnsbar_fbIO); 
            LvG = pz*z*g2; 
            Aclf =  [LvG, -1];
            Bclf = -obj.gama*V - LvF; 
            
            %%%%%%%%%%%%%%%%%%%% CBF constraint construction %%%%%%%%%%%%
            hns = Fns ;% obj.Fmin; 
           % hs = Fs; %%%% another option uses this
            %%%%%%%%%%% dh = f2 + g2*uns > - beta*h, Acbf*u < bcbf
            Acbf = [-g2, 0];
            Bcbf = f2 + obj.beta*hns;
            A = [Aclf; Acbf]; 
            ubA = [Bclf; Bcbf];
            
            lbA = -inf*ones(size(ubA));
            lb = [-obj.ddLmax; 
                  -inf]; 
            ub = [obj.ddLmax; 
                  inf];
            %%%%%%%%%%%% QP construction %%%%%%%%%%%%%%%%%%%%%%%%
            %%%% min |uns - unsdes|^2 + p*delta^2
            p = 1;
            H = [1, 0;
                 0, p];
            G = [-u_ns_des;
                 0];
            nWSR = 10000;
%             options = qpOASES_options('maxIter',nWSR,'printLevel',1);
%             [sol,fval,exitflag, numiter] = qpOASES(H, G, A, lb, ub, lbA,ubA,options);
            options = optimoptions('quadprog','Display','off');
            [usol,fval,exitflag, numiter] = quadprog(H,G, A, ubA, [], [], lb, ub,[],options);
            delta = sol(2); 
            u = [us; sol(1)];
            dV = LvF + LvG*sol(1); 
            cbfH = Acbf*sol - Bcbf;
            cbfH = [cbfH; 0; 0];
        end
        
        function [u, V, dV, delta, cbfH, zdesStruct, FdesStruct, exitflag] = DSPUnitedBackSteppingCLFCBF(obj, t)
            %%% combine the input, using CBF for stance force as well. 
            rs = obj.polar.rs;            rns = obj.polar.rns;
            drs = obj.polar.drs;          drns = obj.polar.drns;
            sLs = obj.polar.sLs;          sLns = obj.polar.sLns;
            dsLs = obj.polar.dsLs;        dsLns = obj.polar.dsLns;
            qs = obj.polar.qs;            qns = obj.polar.qns;
            dqs = obj.polar.dqs;          dqns = obj.polar.dqns;
            Ls = obj.polar.Ls;            Lns = obj.polar.Lns;

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
            Fs = obj.springForce(Ls, sLs, dsLs);
            Fns = obj.springForce(Lns, sLns, dsLns);
            
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
            gs = obj.D(Ls,obj.Dparam); fs = obj.K(Ls,obj.Kparam)*dsLs - obj.D(Ls,obj.Dparam)*ddrs;
            gns = obj.D(Lns,obj.Dparam); fns = obj.K(Lns,obj.Kparam)*dsLns - obj.D(Lns,obj.Dparam)*ddrns;
            
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
            %%%% dV < -gama*V + delta; LvF + LvG*u_sum <  -gama*V + delta;  Aclf*u < bclf
            %%%% u_sum = cos(qs)*gs*us + cos(qns)*gns*uns

            LvF = dVeta + pz*z*(f_sum - dFsumbar_fbIO); 
            LvG = pz*z*g_sum; 
            Aclf =  [LvG*cos(qs)*gs, LvG*cos(qns)*gns, -1e+3];
            Bclf = -obj.gama*V - LvF; 
            
            
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
                
                hs = (c*FdesStruct.Fsdes + deltaF)^2 - (Fs - FdesStruct.Fsdes)^2;
                Acbf_s = [2*(Fs-FdesStruct.Fsdes)*gs, 0, 0];
                Bcbf_s = c^2*2*FdesStruct.Fsdes*FdesStruct.dFsdes + ...
                         2*c*deltaF*FdesStruct.dFsdes - ...
                         2*(Fs - FdesStruct.Fsdes)*(fs - FdesStruct.dFsdes) + ...
                         obj.beta*hs;
                
                hns = Fns;  % obj.Fmin;
                % dh = fns + gns*uns > - beta*h, Acbf*u < bcbf
                Acbf_ns = [0, -gns, 0];
                Bcbf_ns = fns + obj.beta*hns;
                
%                 hns = (c*FdesStruct.Fnsdes + deltaF)^2 - (Fns - FdesStruct.Fnsdes)^2;
%                 Acbf_ns = [0, 2*(Fns - FdesStruct.Fnsdes)*gs, 0];
%                 Bcbf_ns = c^2*2*FdesStruct.Fnsdes*FdesStruct.dFnsdes + ...
%                           2*c*deltaF*FdesStruct.dFnsdes - ...
%                           2*(Fns - FdesStruct.Fnsdes)*(fns - FdesStruct.Fnsdes) + ...
%                           obj.beta*hns;
                %%% log the admissible force set: 
                obj.GRFsbound  = [obj.GRFsbound; [t, (1-c)*FdesStruct.Fsdes - deltaF, (1+c)*FdesStruct.Fsdes + deltaF]]; 
%                 obj.GRFnsbound = [obj.GRFnsbound; [t, (1-c)*FdesStruct.Fnsdes - deltaF, (1+c)*FdesStruct.Fnsdes + deltaF]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0]]; 
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
                obj.GRFsbound  = [obj.GRFsbound; [t, (1-obj.c_relax_DSP)*Fsdes - deltaF, (1+obj.c_relax_DSP)*Fsdes + deltaF]]; 
                obj.GRFnsbound = [obj.GRFnsbound; [t, 0, 0]]; 
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
            Hu = [a1^2, a1*a2; a1*a2, a2^2];
            H = blkdiag(Hu, p);
            G = [-2*u_sum_des*a1; -2*u_sum_des*a2;0];
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
            
            swingZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.z, tInDomain);
            swingdZdes = interp1(obj.swingZbehavior.t, obj.swingZbehavior.dz, tInDomain);
%             terrainZ = interp1(obj.terrain.x, obj.terrain.zfilter, swingX);
         
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
            if obj.useSwingFootDetection && ...
               (obj.zsw < obj.detectionHeight && obj.xsw > obj.xcom) && ...
               obj.stepCnt > 9
                r = sqrt(obj.xsw^2 + (obj.zsw-0.02)^2);
                theta = asin(obj.zsw/r);
            else
                theta = interp1(obj.terrain.x, obj.terrain.theta, x);
            end
            obj.LIP.theta = theta;        
            tliftOff = obj.TimeStamp(end);

            tNow = t - tliftOff;
            if isfield(obj.polar,'Fs')
                obj.LIP.LIPacc = (obj.polar.Fs*sin(obj.polar.qs) + obj.polar.Fns*sin(obj.polar.qns))/obj.m;
            end
            [stepLdes, dstepLdes] = obj.LIP.LIPbasedController(t, x, x2f, dx, obj.TS - tNow);
            %%% smoothing from the current swing foot position
            stepLengthPre = - obj.stepLengthSequence(end);
            % t = obj.t;
           
            cNow = interp1(obj.dumpVec, obj.smfVec, tNow)'; %cNow from 0 ->1
            dcNow = interp1(obj.dumpVec, obj.dsmfVec, tNow)'; %cNow from 0 ->1

            if ~isempty(obj.terrain.avoidStepping)
                N = size(obj.terrain.avoidStepping,1); 
                for i = 1:N
                    region = obj.terrain.avoidStepping(i,:); 
                    if obj.newAvoid == 0 %%% was not in the avoid region 
                        if tNow>obj.TS*5/6
                            swingXdes = obj.stanceFootX + stepLdes;
                            if swingXdes > region(1) && swingXdes<region(2)
                                %                             if swingXdes> region(1) + obj.terrain.avoidSize
                                %                                 swingXdes = region(2);
                                %                             else
                                obj.newAvoid = i; %% become inside the region
                                swingXdes = region(1);
                                %  end
                                stepLdes = swingXdes - obj.stanceFootX;
                                dstepLdes = 0;
                            end
                        end
                    else
                        swingXdes = obj.terrain.avoidStepping(obj.newAvoid,1);                 
                        stepLdes = swingXdes - obj.stanceFootX;
                    end
                end 
            end
            stepL = stepLdes.*cNow + (1-cNow).*stepLengthPre;
            dstepL = dstepLdes.*cNow + stepLdes.*dcNow + (-dcNow).*stepLengthPre; %% important for swing Z control.
        end
    end 
    
    methods  %%% gen desired  
        function genDesiredbehaviorUnexpDownstep(obj, downstep)
            obj.terrain.genUnexpDownstep(downstep);
            x = obj.terrain.x;
            zdes = obj.z0*ones(size(obj.terrain.dz)); 
            dzdxdes = zeros(size(obj.terrain.dz));
            ddzdxdes = zeros(size(obj.terrain.dz));
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorExpDownstep(obj, downstep)
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
            x = -1:0.1:30;
            zdes = obj.z0*ones(size(x));
            dzdes = 0*ones(size(x));
            ddzdes = 0*ones(size(x));
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdes, 'ddzdes',ddzdes );       
%              zdes = 0.9338+0.1*sin(t);
%             dzdes = 0.1*cos(t);
%             ddzdes = -0.1*sin(t);
%             obj.desiredbehavior = struct('t', t, 'zdes', zdes, 'dzdes', dzdes, 'ddzdes',ddzdes );
        end
        
        function genDesiredbehaviorSlopeFix(obj) 
            obj.terrain.genSlope; 
            %%%%%%%% flat %%%%%%
            x = 0:0.1:3;
            zdes = obj.z0+x/3*0.5;
            dzdxdes  = 0.5/3*ones(size(x));
            ddzdxdes = 0.5/3*ones(size(x));            
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorSlopeDeg(obj, deg)
            obj.terrain.genSlopeDeg(deg2rad(deg));
            x =  obj.terrain.x;
            zdes = obj.z0+obj.terrain.zfilter;
            dzdxdes  = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz)); 
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorUpStairs(obj) 
            obj.terrain.genStairs; 
            %%%%%%%% flat %%%%%%
            x =  obj.terrain.x;
             zdes = obj.z0+obj.terrain.zfilter;
            dzdxdes  = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz));          
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorUpUpStairs(obj) 
            obj.terrain.genUpStairs; 
            %%%%%%%% flat %%%%%%
            x =  obj.terrain.x;
            zdes = obj.z0+obj.terrain.zfilter;
            dzdxdes  = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz));          
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
         end
        
        function genDesiredbehaviorSinu(obj) 
            obj.terrain.genSinu2; 
            %%%%%%%% flat %%%%%%
            x =  obj.terrain.x;
            zdes =obj.z0+obj.terrain.zfilter;
            dzdxdes  = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz));          
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
        
        function genDesiredbehaviorArbitrary(obj) 
            %obj.terrain.genSample; 
            obj.terrain.genSampleRough; 
            %%%%%%%% flat %%%%%%
            x = obj.terrain.x;
            zdes =obj.z0+obj.terrain.zfilter;
            dzdxdes  = obj.terrain.dzfilter;
            ddzdxdes = zeros(size(obj.terrain.dz));            
            %%%%%%%% slope
            obj.desiredbehavior = struct('x', x, 'zdes', zdes, 'dzdes', dzdxdes, 'ddzdes',ddzdxdes);
        end
    end
    
    methods %%% assist
        function [Fdes, dFdes] = getDesiredF(obj,t)
            persistent passedDSP2
            if isempty(passedDSP2)
                passedDSP2 = false;
            end
            
            if obj.NcontactLegs == 1
                if obj.isDownstep
                    phase = 'phase1';
                    switch obj.downstepStep
                        case 1
                            phase = 'phase1';
                        case 2
                            phase = 'phase2';
                        case 3
                            phase = 'phase3';
                        otherwise
                            disp("INVALID DOWNSTEPSTEP");
                    end
%                     tTmp = clamp(obj.stepTime,0.0,obj.downstepBeziers.unexp25.(phase).timeMax_SSP);
%                     t_norm = (tTmp - 0.0) / ...
%                         (obj.downstepBeziers.unexp25.(phase).timeMax_SSP - 0.0);
                    
                    tTmp = clamp(obj.stepTime,0.0,obj.nominalBeziers.timeMax_SSP);
                    t_norm_nominal = (tTmp - 0.0) / ...
                        (obj.nominalBeziers.timeMax_SSP - 0.0);
                    
                    % eval beziers
                    Fdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_grf_SSP,t_norm_nominal);
                    dFdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dgrf_SSP,t_norm_nominal);
                    % for now, try on the third step the nominal GRFs
                    if obj.downstepStep == 3
                        Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,t_norm_nominal);
                        dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm_nominal);
                    end
                else
                    % SSP: Normalize time for bezier 
                    tTmp = clamp(obj.stepTime,0.0,obj.nominalBeziers.timeMax_SSP);
                    t_norm = (tTmp - 0.0) / ...
                        (obj.nominalBeziers.timeMax_SSP - 0.0);
                    % eval beziers
                    Fdes = bezier2(obj.nominalBeziers.bv_grf_SSP,t_norm);
                    dFdes = bezier2(obj.nominalBeziers.bv_dgrf_SSP,t_norm);
                end
            else 
                % DSP
                if obj.isDownstep
                    phase = 'phase1';
                    switch obj.downstepStep
                        case 1
                            phase = 'phase1';
                        case 2
                            phase = 'phase2';
                        case 3
                            phase = 'phase3';
                        otherwise
                            disp("INVALID DOWNSTEPSTEP");
                    end
%                     tTmp = clamp(obj.stepTime,0.0,obj.downstepBeziers.unexp25.(phase).timeMax);
%                     t_norm = (tTmp - obj.downstepBeziers.unexp25.(phase).timeMax_SSP) / ...
%                         (obj.downstepBeziers.unexp25.(phase).timeMax - obj.downstepBeziers.unexp25.(phase).timeMax_SSP);
                    tTmp = clamp(obj.stepTime,0.0,obj.nominalBeziers.timeMax);
                    t_norm = (tTmp - obj.nominalBeziers.timeMax_SSP) / ...
                        (obj.nominalBeziers.timeMax - obj.nominalBeziers.timeMax_SSP);
                    % eval beziers
                    Fdes = zeros(2,1);
                    dFdes = zeros(2,1);
                    % sw st naming convention 
                    Fdes(1) = bezier2(obj.downstepBeziers.unexp25.(phase).bv_grf_DSP_sw,t_norm);
                    Fdes(2) = bezier2(obj.downstepBeziers.unexp25.(phase).bv_grf_DSP_st,t_norm);
                    dFdes(1) = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dgrf_DSP_sw,t_norm);
                    dFdes(2) = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dgrf_DSP_st,t_norm);
                    % for now, create spline from last desired GRF to 0
                    if obj.downstepStep == 2
                        if ~passedDSP2
                            % spline t_norm now to 1, fGRF now to 0
                            tc = t_norm;
                            tf = 1.0;
                            Fc = obj.FdesPrev.Fsdes;
                            Ff = 0.0;
                            
                            xData = 0:0.01:1;
                            zData = ((Ff - Fc)/(tf - tc))*(xData - tc) + Fc;
                            nMid = round(length(zData)/2);
                            bv_0 = [zData(1) zData(1) zData(nMid) zData(end) zData(end)];
                            fun = @(x) costFcnBezier(x,xData,zData);
                            obj.bv_SSP2.bv_grf_DSP_st = fmincon(fun,bv_0);
                            
                            passedDSP2 = true;
                        end
                        Fdes(2) = bezier2(obj.bv_SSP2.bv_grf_DSP_st,t_norm);
                        dFdes(2) = (obj.bv_SSP2.bv_grf_DSP_st(end) - obj.bv_SSP2.bv_grf_DSP_st(1))*obj.nominalBeziers.timeMax_SSP;
                    end
                    % for now, try on the third step the nominal GRFs
                    if obj.downstepStep == 3
                        Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,t_norm);
                        Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,t_norm);
                        dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                        dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
                    end
                else
                    % DSP: Normalize time for bezier, DSP from 0 to 1
                    tTmp = clamp(obj.stepTime,0.0,obj.nominalBeziers.timeMax);
                    t_norm = (tTmp - obj.nominalBeziers.timeMax_SSP) / ...
                        (obj.nominalBeziers.timeMax - obj.nominalBeziers.timeMax_SSP);
                    % eval beziers
                    Fdes = zeros(2,1);
                    dFdes = zeros(2,1);
                    Fdes(1) = bezier2(obj.nominalBeziers.bv_grf_DSP_sw,t_norm);
                    Fdes(2) = bezier2(obj.nominalBeziers.bv_grf_DSP_st,t_norm);
                    dFdes(1) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_sw,t_norm);
                    dFdes(2) = bezier2(obj.nominalBeziers.bv_dgrf_DSP_st,t_norm);
                end
            end
            obj.FdesPrev.Fdes = Fdes;
            obj.FdesPrev.dFdes = dFdes;
        end
        
        function [zdes, dzdes, ddzdes] = getDesiredZ(obj, t)
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                if obj.isDownstep
                    %%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%
                    %%% DOWNSTEPS
                    switch obj.downstepStep
                        case 1
                            phase = 'phase1';
                            % as we know the splines from VLO to end of
                            % DSP, the phase considered here slightly is
                            % shifted (linearly) by 42%
                            if obj.stepTime > obj.nominalBeziers.timeMax
                                t_norm = 1;
                            else
                                t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                            end
                            t_norm_VLO = (t_norm - 0.42)/(1.00 - 0.42);
                            zdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,t_norm_VLO)+0.01;
                            dzdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dzcom,t_norm_VLO);
                            ddzdes = 0.0*bezier2(obj.downstepBeziers.unexp25.(phase).bv_ddzcom,t_norm_VLO);
                            
                        case 2
                            phase = 'phase2';
                            if obj.stepTime > obj.nominalBeziers.timeMax
                                t_norm = 1;
                            else
                                t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                            end
                            zdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,t_norm)+0.01;
                            dzdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dzcom,t_norm);
                            ddzdes = 0.0*bezier2(obj.downstepBeziers.unexp25.(phase).bv_ddzcom,t_norm);
                            return; 
                            
                        case 3
                            phase = 'phase3';
                            if obj.stepTime > obj.nominalBeziers.timeMax
                                t_norm= 1;
                            else
                                t_norm= obj.stepTime/obj.nominalBeziers.timeMax;
                            end
                            t_norm_VLO = (t_norm - 0)/(0.42 - 0);
                            
                            zdes = bezier2(obj.downstepSplines.unexp25.(phase).bv_zcom,t_norm_VLO)+0.01;
                            dzdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dzcom,t_norm_VLO);
                            ddzdes = 0.0*bezier2(obj.downstepBeziers.unexp25.(phase).bv_ddzcom,t_norm_VLO);
                            return;                            
                    end
                else
                    if obj.stepTime > obj.nominalBeziers.timeMax
                        t_norm = 1;
                    else
                        t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
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
                        ddzdes = 0.0*bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                    else
                        factor = 0.5;
                        zdes = factor*(bezier2(obj.nominalBeziers.bv_zcom,t_norm)-zdesMean) + zdesMean;
                        dzdes = factor*bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
                        ddzdes = 0.0*bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                    end
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
%             obj.zdesPrev.zdes = zdes;
%             obj.zdesPrev.dzdes = dzdes;
%             obj.zdesPrev.ddzdes = ddzdes;
        end
        
        function [zdes, dzdes, ddzdes] = getDesiredZUnknown(obj, t)
            persistent passedDSP1 passedSSP3
            if isempty(passedDSP1)
                passedDSP1 = false;
                passedSSP3 = false;
            end
            
            x = obj.polar.x; 
            dx = obj.polar.dx;
            if obj.useHumanZ
                if obj.isDownstep
                    %%%%%%%%%%%%%%%%%
                    %%%%%%%%%%%%%%%%%
                    %%% DOWNSTEPS
                    %%% phase1: SSP: we go down with the same desired
                    %%% velocity as last instant, DSP: we recover back from
                    %%% the starting point we displace the bezier
                    %%%
                    %%% phase2: SSP: follow bezier, displaced by end point,
                    %%% DSP: follow bezier, displaced by end point of SSP
                    %%%
                    %%% phase3: SSP: construct bezier of duration to go
                    %%% from current, with current velocity, to desired
                    %%% (pre-DSP point)
                    switch obj.downstepStep
                        case 1
                            phase = 'phase1';
                            if obj.NcontactLegs == 1
                                if obj.stepTime - obj.downstepDuration > obj.nominalBeziers.timeMax
                                    t_norm_nominal = 1;
                                else
                                    t_norm_nominal = obj.stepTime / obj.nominalBeziers.timeMax;
                                end
                                % Just push it down to the desired asap
                                zdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,obj.TS/(obj.TS+obj.TD));
%                                 dzdes = min(bezier2(obj.nominalBeziers.bv_dzcom,0:0.01:1));
                                dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm_nominal);
                                ddzdes = 0.0;
                                return;
                            else
                                % Displace t_norm by the time it took to go
                                % from detection to impact and then use a
                                % displaced deltaZ so we start on the bz
%                                 if ~passedDSP1
                                    obj.downstepDuration   = t - obj.downstepTime;
                                    obj.downstepHeightOffset = obj.polar.z;
%                                     passedDSP1 = true;
%                                 end
                                % t_norm from nominal walking, we use this
                                % now for the velocity
                                if obj.stepTime - obj.downstepDuration > obj.nominalBeziers.timeMax
                                    t_norm_nominal = 1;
                                else
                                    t_norm_nominal = obj.stepTime / obj.nominalBeziers.timeMax;
                                end
                                % t_norm for downstep
                                if obj.stepTime - obj.downstepDuration > obj.nominalBeziers.timeMax
                                    t_norm = 1;
                                else
                                    t_norm = (obj.stepTime - obj.downstepDuration) ...
                                        /obj.nominalBeziers.timeMax;
                                end
                                zdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,t_norm) - ...
                                    bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,0.0) + obj.downstepHeightOffset;
%                                 dzdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dzcom,t_norm);
                                dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm_nominal);
                                ddzdes = 0.0*bezier2(obj.downstepBeziers.unexp25.(phase).bv_ddzcom,t_norm);
                                return;
                            end
                            
                        case 2
                            phase = 'phase2';
                            if obj.stepTime > obj.nominalBeziers.timeMax
                                t_norm = 1;
                            else
                                t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
                            end
                            zdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,t_norm) - ...
                                bezier2(obj.downstepBeziers.unexp25.(phase).bv_zcom,0.0) + obj.downstepHeightOffset;
%                             dzdes = bezier2(obj.downstepBeziers.unexp25.(phase).bv_dzcom,t_norm);
                            dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
                            ddzdes = 0.0*bezier2(obj.downstepBeziers.unexp25.(phase).bv_ddzcom,t_norm);
                            return; 
                            
                        case 3
                            phase = 'phase3';
                            if obj.stepTime > obj.nominalBeziers.timeMax
                                t_norm_nominal = 1;
                            else
                                t_norm_nominal = obj.stepTime/obj.nominalBeziers.timeMax;
                            end
                            if obj.stepTime > obj.downstepBeziers.unexp25.(phase).timeMax
                                t_norm = 1;
                            else
                                t_norm = obj.stepTime/obj.downstepBeziers.unexp25.(phase).timeMax;
                            end
                            if ~passedSSP3
                                % construct Bezier polynomials from the
                                % current position to the end of nominal
                                % DSP so we can use this for the whole
                                % downstep phase 3 (now we also reset to
                                % not-downstep if swing leg above and DSP
                                LOfrac = 2*obj.downstepDuration;
                                
                                zcom_SSP = bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1);
                                dzcom_SSP = bezier2(obj.nominalBeziers.bv_zcom,0:0.01:1);
%                                 zcom_before  = linspace(obj.polar.z,...
%                                     bezier2(obj.nominalBeziers.bv_zcom,0.0),...
%                                     round(100*LOfrac/obj.TS));
%                                 dzcom_before = linspace(obj.polar.dz,...
%                                     bezier2(obj.nominalBeziers.bv_dzcom,0.0),...
%                                     round(100*LOfrac/obj.TS));
                                zcom_before  = linspace(obj.zdesPrev.zdes,...
                                    bezier2(obj.nominalBeziers.bv_zcom,0.0),...
                                    round(100*LOfrac/obj.TS));
                                dzcom_before = linspace(obj.zdesPrev.dzdes,...
                                    bezier2(obj.nominalBeziers.bv_dzcom,0.0),...
                                    round(100*LOfrac/obj.TS));
                                
                                zcom_total = [zcom_before zcom_SSP];
                                dzcom_total = [dzcom_before dzcom_SSP];
                                xData = linspace(0,1,length(zcom_total));
                                
                                nMid = round(length(zcom_total)/2);
                                bv_0 = [zcom_total(1) zcom_total(1) zcom_total(nMid) 0 0 zcom_total(end) zcom_total(end)];
                                fun = @(x) costFcnBezier(x,xData,zcom_total);
                                obj.bv_SSP3.bv_zcom = fmincon(fun,bv_0);
                                
                                bv_0 = [dzcom_total(1) dzcom_total(1) dzcom_total(nMid) 0 0 dzcom_total(end) dzcom_total(end)];
                                fun = @(x) costFcnBezier(x,xData,dzcom_total);
                                obj.bv_SSP3.bv_dzcom = fmincon(fun,bv_0);
                                passedSSP3 = true;
                            end
%                             zdes = bezier2(obj.bv_SSP3.bv_zcom,t_norm);
%                             dzdes = bezier2(obj.bv_SSP3.bv_dzcom,t_norm);
%                             ddzdes = 0.0*bezier2(obj.bv_SSP3.bv_dzcom,t_norm);
                            zdes = bezier2(obj.bv_SSP3.bv_zcom,t_norm_nominal);
                            dzdes = bezier2(obj.nominalBeziers.bv_dzcom,t_norm_nominal);
                            ddzdes = 0.0*bezier2(obj.bv_SSP3.bv_dzcom,t_norm_nominal);
                            return;

                        otherwise
                            disp("INVALID DOWNSTEPSTEP");
                            zdes = 0;
                            dzdes = 0;
                            ddzdes = 0;
                            return;
                            
                    end
                else
                    if obj.stepTime > obj.nominalBeziers.timeMax
                        t_norm = 1;
                    else
                        t_norm = obj.stepTime/obj.nominalBeziers.timeMax;
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
                        ddzdes = 0.0*bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                    else
                        factor = 0.5;
                        zdes = factor*(bezier2(obj.nominalBeziers.bv_zcom,t_norm)-zdesMean) + zdesMean;
                        dzdes = factor*bezier2(obj.nominalBeziers.bv_dzcom,t_norm);
                        ddzdes = 0.0*bezier2(obj.nominalBeziers.bv_ddzcom,t_norm);
                    end
                end
            else
                zdes = interp1(obj.desiredbehavior.x, obj.desiredbehavior.zdes, x);
                dzdes = dx*interp1(obj.desiredbehavior.x, obj.desiredbehavior.dzdes, x);
                ddzdes = 0*interp1(obj.desiredbehavior.x, obj.desiredbehavior.ddzdes, x);
            end
%             obj.zdesPrev.zdes = zdes;
%             obj.zdesPrev.dzdes = dzdes;
%             obj.zdesPrev.ddzdes = ddzdes;
        end
        
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
            
%             vimpact = obj.impactVel; %0.05;
%             %%%%% add velocity adjustment
%             N = length(Tvec);
%             TvecQuad1 = Tvec(1:floor(N/4));
%             TvecQuad2 = Tvec(ceil(N/4):floor(N/2));
%             TvecHalf2 = Tvec(ceil(N/2):N);
%             
%             vel_1 = vimpact/(Tf/4)*TvecQuad1;
%             vel_2 = vimpact - vimpact/(Tf/4)*(TvecQuad2 - Tf/4);
%             vel_3 = -vimpact/(Tf/2)*(TvecHalf2-Tf/2);
%             addZvel =  -[vel_1, vel_2, vel_3];
%             addZvel = addZvel(1:N);
%             
%             swingZvel = swingZvel + addZvel;
%             swingZpos =  swingZ + cumsum(addZvel)*period;
%             
          
            obj.swingZbehavior = struct('t', Tvec, 'z', swingZpos, 'dz', swingZvel);
        end
    end
end