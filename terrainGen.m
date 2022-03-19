classdef terrainGen < handle
    %%% Unorganzied code for 
    %%% generating different terrain shapes in 2D simulation environment 
    %%%%%%%%%%%%%%%%%%%% Xiaobin Xiong %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
        z  % terrain height   
        dz % height derivative
        x % horizontal position 
        
        unit = 0.001 
        theta %%% approxi slope
        
        stairL = 0.15;  % typical stair height 
        
        zfilter
        dzfilter
        swingZfilter %%% for swing foot, small filtering
        swingdZfilter
        blendX = 0.1;
        smoothingWindow = 0.5; 
        
        avoidStepping = [];
        avoidSize = 0.05; %0.03
    end
    
    methods
        function obj = terrainGen()
            obj.reset;
        end
        
        function obj = reset(obj) 
            obj.x = -1:obj.unit:0;            
            obj.z = zeros(size(obj.x));
            obj.dz = zeros(size(obj.x));
              obj.zfilter = obj.z; 
            obj.dzfilter = obj.dz; 
              obj.swingZfilter = obj.z; 
            obj.swingdZfilter = obj.dz; 
            
            obj.theta = 0*obj.z; 
        end
        
        function plotTerrain(obj)
            figure,
            subplot(3,1,1)
            plot(obj.x, obj.z, 'r', obj.x, obj.zfilter, 'b', obj.x, obj.swingZfilter, 'k')
            legend('z','z filter', 'swing')
            xlabel('x'),ylabel('z');
            ylim([-0.2, 1])
            daspect([1 1 1])
            subplot(3,1,2)
            plot(obj.x, obj.theta);
            xlabel('x'),ylabel('slope');
            subplot(3,1,3)
            plot(obj.x, obj.dz, 'r',  obj.x, obj.dzfilter, 'b')
            legend('z','z filter')
            title('dz')
            
%             plot(obj.x, obj.z, 'r', obj.x, obj.approxZ, 'b' );
%             ylim([min(obj.dz), max(obj.dz)])
%             legend('z', 'approxZ')
%             xlabel('x'),ylabel('z comp');
        end     
        
        function obj = genSample(obj)
            obj.reset;
            obj.addStraight(0.3); 
            obj.addIncline(1, 0.3); 
            obj.addSinu(2, 0.4); 
            obj.addStair(0.3,0.2);
            obj.addStair(0.3,-0.2);
            obj.addDecline(1, 0.2);
            obj.addStraight(0.3);
            obj.smoothingFilter(); 
            obj.plotTerrain();
        end
        
        function obj = genSampleRough(obj) 
            obj.reset;
            obj.smoothingWindow = 0.5; 
            obj.addStraight(0.3);
            obj.addIncline(0.4, 0.2);
            obj.addIncline(0.4, -0.2);
            obj.addStair(0.3,0.2);
            obj.addStair(0.3,0.2);            
            obj.addStair(0.3,0.2);
            obj.addStair(0.3,-0.2);
            obj.addStair(0.3,-0.2);
            obj.addStair(0.3,-0.2);
            obj.addSinu(2, -0.2);
            obj.addDecline(1, 0.2);
            obj.addStraight(0.3);
            obj.smoothingFilter();            
            obj.addNoise(0.11); 
            obj.plotTerrain();
        end 
        
        function obj = genSampleRough2(obj) 
            obj.reset;
            obj.smoothingWindow = 0.5; 
            obj.addStraight(0.5);
            obj.addIncline(0.4, 0.3);
            obj.addIncline(0.4, -0.3);
            obj.addStair(0.35,0.2);
            obj.addStair(0.35,0.2);            
            obj.addStair(0.35,0.2);
            obj.addStair(0.35,0.2);
            obj.addStair(0.35,0.2);
            obj.addStair(0.35,0.2);
            obj.addSinu(2, -0.2);
            obj.addDecline(1, 0.4);
            obj.addStraight(0.6);
            obj.smoothingFilter();            
            % obj.addNoise(0.1); 
            obj.plotTerrain();
        end 
        
        function obj = genSampleRough3(obj)
            obj.reset;
            obj.smoothingWindow = 0.5;
            obj.addStraight(0.5);
            for i = 1:8
                obj.addStair(0.35,0.2);
            end
            obj.addStraight(1);

            obj.smoothingFilter();
            % obj.addNoise(0.1);
            obj.plotTerrain();
        end
       
        function obj = genSampleRough4(obj)
            obj.reset;
            obj.smoothingWindow = 0.5;      
            obj.addStraight(0.5);
            obj.addSinu(2, -0.3);
            obj.addStair(0.35,0.2);
            obj.addStair(0.35,-0.2);
             obj.addSinu(2, 0.3);
%             for i = 1:8
%                 obj.addStair(0.35,0.2);
%             end
            obj.addStraight(1);

            obj.smoothingFilter();
            obj.addNoise(0.05);
            obj.plotTerrain();
        end
       
        function obj = genFlatThenSlope(obj)
            obj.reset;
            obj.addStraight(0.3); 
            obj.addIncline(3, 0.8); 
            obj.plotTerrain();
        end
        
        function obj = genSlope(obj)
            obj.reset;
            obj.addIncline(3, 0.5); 
            obj.plotTerrain();
        end
        
        function obj = genSinu(obj)
            obj.reset;
            obj.addSinu(3, -0.3); 
            obj.addSinu(3, 0.3); 
            obj.smoothingFilter();
            obj.plotTerrain();
         end
        
        function obj = genSinu2(obj)
             obj.reset;
             obj.addSinu(3, -0.6);
             obj.addSinu(3, 0.6);
             obj.smoothingFilter();
             obj.plotTerrain();
         end
        
        function obj = genSlopeDeg(obj, slope)
            obj.reset;
            for i = 1:length(slope)
                obj.addIncline(3, 3*tan(slope(i)));
            end
            obj.smoothingFilter();
            obj.plotTerrain();
        end
        
        function obj = genUnexpDownstep(obj, downstep)
%             downstepCenter = 5.2; % Human
            downstepCenter = 5.0; % Cassie
            downstepLength = 0.5;
            
            obj.reset;
            obj.addStraight(10);
            obj.smoothingFilter();
            
            [~,idx0] = min(abs(obj.x - (downstepCenter-downstepLength/2)));
            [~,idxf] = min(abs(obj.x - (downstepCenter+downstepLength/2)));
            downstep = [zeros(1,idx0-1) -downstep*ones(1,idxf-idx0+1) zeros(1,length(obj.z)-idxf)];
            obj.z = obj.z + downstep;
            
%             obj.plotTerrain();
        end
        
        function obj = genExpDownstep(obj,downstep)
            downstepCenter = 7.0;
            downstepLength = 0.9;
            
            obj.reset;
            obj.addStraight(downstepCenter-downstepLength/2);
            obj.addStair(downstepLength,-downstep);
            obj.addStair(downstepLength,downstep);
            obj.addStraight(10-(downstepCenter+downstepLength));
            obj.smoothingFilter();
            
            obj.plotTerrain();
        end
        
        function obj = genFlat(obj) 
            obj.x = -1:obj.unit:10;
            obj.z = zeros(size(obj.x)); 
            obj.theta = obj.z*0; 
            obj.zfilter = obj.z;
            obj.smoothingFilter();
        end 
        
        function obj = genStairs(obj)
            obj.reset;
            obj.smoothingWindow = 0.5;
            obj.addStraight(0.2);
            obj.addStair(0.25,0.2);
            obj.addStair(0.25,0.2);
            obj.addStair(0.25,0.2);
            obj.addStair(0.25,0.2);
            obj.addStair(0.25,0.2);
            obj.addStair(0.25,-0.2);
            obj.addStair(0.25,-0.2);
            obj.addStair(0.25,-0.2);
            obj.addStair(0.25,-0.2);
            obj.addStair(0.25,-0.2);
            obj.smoothingFilterStair();
            obj.plotTerrain();
        end
        
        function obj = genUpStairs(obj)
            obj.reset;
            obj.smoothingWindow = 0.5;
            obj.addStraight(0.2);
            for i = 1:10
            obj.addStair(0.25,0.15);
            end
            obj.smoothingFilterStair();
            obj.plotTerrain();
        end
    end 
    
    methods 
        function obj = addSinu(obj, L, H)
            vec = obj.unit:obj.unit:L;
            obj.x = [obj.x, obj.x(end)+vec ];
            obj.z = [obj.z, obj.z(end)+H*(sin(pi/2+vec/L*2*pi)-1)];
            obj.dz = [obj.dz, 2*pi/L*H*cos(pi/2+vec/L*2*pi)];
            obj.theta = [obj.theta, H*2*pi/L*cos(pi/2+vec/L*2*pi)];
            obj.zfilter = [obj.zfilter, obj.zfilter(end)+H*(sin(pi/2+vec/L*2*pi)-1)];
            obj.dzfilter = [obj.dzfilter, 2*pi/L*H*cos(pi/2+vec/L*2*pi)];
        end
        
        function obj = addIncline(obj, L, H)              
            vec = obj.unit:obj.unit:L;
            xvec = obj.x(end) + vec; 
            zvec = obj.z(end) + H*vec/L;
            dzvec =  H/L*ones(size(vec));
            obj.x = [obj.x, xvec ];
            obj.z = [obj.z, zvec];
            obj.dz = [obj.dz, dzvec];

            obj.zfilter = [obj.zfilter, obj.zfilter(end)+H*vec/L];
            obj.dzfilter = [obj.dzfilter, H/L*ones(size(vec))];
            obj.theta = [obj.theta, atan2(H, L)*ones(size(vec))]; 
        end
        
        function obj = addDecline(obj, L, H)
            obj = obj.addIncline(L, -H);
        end
        
        function obj = addStair(obj, L, H)
            vec = 0.00001:obj.unit:L;
         %   vec = obj.unit:obj.unit:L;
            if H >0 %% up
            obj.avoidStepping = [obj.avoidStepping;
                                 obj.x(end)-obj.avoidSize, obj.x(end)+obj.unit];
            else %%% down
                  obj.avoidStepping = [obj.avoidStepping;
                                 obj.x(end)-obj.unit, obj.x(end)+obj.avoidSize];
            end 
            obj.x = [obj.x, obj.x(end)+vec ];
            obj.z = [obj.z, obj.z(end)+H*ones(size(vec))];
            obj.dz = [obj.dz, zeros(size(vec))];
            
            Nback = round(obj.stairL/obj.unit); 
            vecSMF = obj.unit:obj.unit:obj.stairL*2; 
            deltaZ = obj.zfilter(end) - obj.zfilter(end-Nback); 
            
            zSMF = (H+deltaZ)*smf(vecSMF, [obj.stairL*2/8, obj.stairL*2/8*7]);
            dzSMF = [0, diff(zSMF)/obj.unit]; 
            obj.zfilter = obj.zfilter(1:end-Nback); 
            obj.dzfilter = obj.dzfilter(1:end-Nback); 
            obj.zfilter = [obj.zfilter, obj.zfilter(end) + zSMF];
            obj.dzfilter = [obj.dzfilter,  dzSMF];

            Nfilter = length(obj.zfilter); 
            obj.zfilter = [obj.zfilter, obj.zfilter(end)*ones(1, length(obj.z)- Nfilter)]; 
            obj.dzfilter = [obj.dzfilter, zeros(1, length(obj.z)- Nfilter)]; 

            obj.theta = [obj.theta, zeros(size(vec))]; 
        end
        
        function obj = addStairSraightSlope(obj, L, H)
            vec = 0.00001:obj.unit:L;
            vec = obj.unit:obj.unit:L;

            obj.x = [obj.x, obj.x(end)+vec ];
            obj.z = [obj.z, obj.z(end)+H*ones(size(vec))];
            obj.dz = [obj.dz, zeros(size(vec))];
            
            Nback = round(obj.stairL/obj.unit); 
            vecSlope = obj.unit:obj.unit:obj.stairL*2; 
            deltaZ = obj.zfilter(end) - obj.zfilter(end-Nback); 
            obj.zfilter = obj.zfilter(1:end-Nback); 
            obj.dzfilter = obj.dzfilter(1:end-Nback); 
            obj.zfilter = [obj.zfilter, obj.zfilter(end) + (H+deltaZ)/(obj.stairL*2)*vecSlope];
            obj.dzfilter = [obj.dzfilter,  (H+deltaZ)/(obj.stairL*2)*ones(size(vecSlope))];

            Nfilter = length(obj.zfilter); 
            obj.zfilter = [obj.zfilter, obj.zfilter(end)*ones(1, length(obj.z)- Nfilter)]; 
            obj.dzfilter = [obj.dzfilter, zeros(1, length(obj.z)- Nfilter)]; 

            obj.theta = [obj.theta, zeros(size(vec))]; 
        end
        
        function obj = addStraight(obj, L)
            vec = obj.unit:obj.unit:L;
            %%% add straight line before the rough terrain
            obj.x = [obj.x, obj.x(end)+vec ];
            obj.z = [obj.z, obj.z(end)+zeros(size(vec))];
            obj.dz = [obj.dz, zeros(size(vec))];
            obj.zfilter =[obj.zfilter, obj.z(end)+zeros(size(vec))];
            obj.dzfilter =  [obj.dzfilter, obj.dz(end)+zeros(size(vec))];
            obj.theta = [obj.theta, zeros(size(vec))];
        end
        
        function obj = addNoise(obj, MaxH)
            
            xrange = obj.x(1):0.05:obj.x(end); 
            znoise =  (2*rand(1,length(xrange))-1)*MaxH;
            obj.z = obj.z + interp1(xrange, znoise, obj.x);
        end 
    end
    
    methods %%% smoothing via MPC/QP
        function smoothingViaQP(obj)
            sample = 0.05;
            xrange = obj.x(1):sample:obj.x(end);
            x0 = [obj.z(1); obj.dz(1)];
            xF = [obj.z(end); obj.dz(end)];
            xdes = [interp1(obj.x, obj.z, xrange);
                 interp1(obj.x, obj.dz, xrange)];
            A = [1, sample; 1, 0];
            B = [sample^2/2; sample]; 
            N = length(xrange);
            X = sdpvar(size(A,2)*ones(1,N+1),ones(1,N+1));
            u = sdpvar(size(B,2)*ones(1,N),ones(1,N));
            
            Constraints = [X{1} == xdes(:,1)];
            % System Dynamics
            for i = 1:N
                Constraints=[Constraints;
                    X{i+1} == A*X{i} + B*u{i};];
            end
            % ======= Cost Definition ======
            % Running Cost
            Cost=0;
            Q = [1000,0;0,1]; R = 0.1;
            for i=1:N
                Cost = Cost + (X{i}-xdes(:,i))'*Q*(X{i}-xdes(:,i)) + u{i}'*R*u{i};
            end
            solver = 'quadprog';

            % Solve the FTOCP
            options = sdpsettings('verbose',0,'solver',solver);
            Problem = optimize(Constraints,Cost,options);
            Objective = double(Cost);
            
            x_cl = [];
            for i = 1:(N+1)
                x_cl = [x_cl, double(X{i})];
            end
            u_cl = [];
            for i = 1:(N)
                u_cl = [u_cl, double(u{i})];
            end
            zfilter = x_cl(1,1:end-1); 
            dzfilter = x_cl(2,1:end-1); 
            
            obj.zfilter = interp1(xrange, zfilter, obj.x); 
            obj.dzfilter = interp1(xrange, dzfilter, obj.x); 
        end 
        
        function smoothingViaMPC(obj)
            %%% same thing as QP 
            Nhorizton = 10; 
            sample = 0.05;
            xrange = obj.x(1):sample:obj.x(end);
            x0 = [obj.z(1); obj.dz(1)];
            xF = [obj.z(end); obj.dz(end)];
            xdes = [interp1(obj.x, obj.z, xrange);
                interp1(obj.x, obj.dz, xrange)];
            
            xact = xdes;
            
            A = [1, sample; 1, 0];
            B = [sample^2/2; sample];
            Ntotal = length(xrange);
         
            xnow = xdes(:, 1);
            for i = 1:Ntotal-Nhorizton               
                xact(:,i) = xnow;
                X = sdpvar(size(A,2)*ones(1,Nhorizton+1),ones(1,Nhorizton+1));
                u = sdpvar(size(B,2)*ones(1,Nhorizton),ones(1,Nhorizton));
                
                Constraints = [X{1} == xnow];
                for m = 1:Nhorizton
                    Constraints=[Constraints;
                        X{m+1} == A*X{m} + B*u{m};];
                end
                % ======= Cost Definition ======
                % Running Cost
                Cost=0;
                Q = [1000,0;0,1]; R = 0.1;
                for m =1:Nhorizton
                    Cost = Cost + (X{m}-xdes(:,i+m))'*Q*(X{m}-xdes(:,i+m)) + u{m}'*R*u{m};
                end
                solver = 'quadprog';
                
                % Solve the FTOCP
                options = sdpsettings('verbose',0,'solver',solver);
                Problem = optimize(Constraints,Cost,options);
                Objective = double(Cost);
                
                x_cl = [];
                for m = 1:(Nhorizton+1)
                    x_cl = [x_cl, double(X{m})];
                end
                u_cl = [];
                for m = 1:(Nhorizton)
                    u_cl = [u_cl, double(u{m})];
                end
                xnow = A*xnow + B*u_cl(1);
            end
            
            xact(:, Ntotal-Nhorizton+1) = xnow;
            for i = 2:Nhorizton 
                xnow = A*xnow + B*u_cl(i);
                xact(:,Ntotal-Nhorizton + i) = xnow;
            end 
            
            obj.zfilter = interp1(xrange,   xact(1,:), obj.x);
            obj.dzfilter = interp1(xrange,  xact(2,:), obj.x);
        end
        
        function smoothingFilterStair(obj)
            window = round(obj.smoothingWindow/obj.unit);
            obj.zfilter = smooth(obj.z, window, 'lowess');
            obj.dzfilter = [0; diff(obj.zfilter)/obj.unit]; %smooth(obj.dz, window, 'lowess');
            
            window = round(obj.smoothingWindow/2/obj.unit);
            obj.swingZfilter = smooth(obj.z, window, 'lowess');
            obj.swingdZfilter = [0; diff(obj.swingZfilter)/obj.unit]; %smooth(obj.dz, window, 'lowess');
            
          %  obj.plotTerrain();
%             dz = [0; diff(obj.zfilter)/obj.unit]; 
%             figure, plot(obj.x, dz, 'r', obj.x, obj.dzfilter, 'b')
        end
        
        function smoothingFilter(obj)
            window = round(obj.smoothingWindow/obj.unit);
            obj.zfilter = smooth(obj.z, window, 'lowess');
            obj.dzfilter = [0; diff(obj.zfilter)/obj.unit]; %smooth(obj.dz, window, 'lowess');
            
            obj.swingZfilter = obj.zfilter;
            obj.swingdZfilter = obj.dzfilter; %smooth(obj.dz, window, 'lowess');
    
        end
    end
end

