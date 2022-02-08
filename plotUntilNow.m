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


figure; hold on; grid on;
plot(t,obj.isDownstepLog,'b')
plot(t,obj.downstepStepLog,'r')
legend('isDownstep','downstepStep')


figure; hold on; grid on;
plot(t,obj.stepTimelog)


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